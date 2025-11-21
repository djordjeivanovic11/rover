#!/usr/bin/env python3
"""
NavigateToGPS action server.

Accepts GPS waypoints (lat, lon) and forwards them to Nav2's NavigateToPose
action after converting them into the map frame using a local ENU approximation.
"""

import math
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformListener

from urc_msgs.action import NavigateToGPS  # type: ignore

from .gps_converter import GPSConverter


class GPSNavigatorNode(Node):
    def __init__(self):
        super().__init__("gps_navigator")

        # Parameters
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("min_gps_quality_m", 10.0)

        self._goal_timeout = float(
            self.get_parameter("goal_timeout_sec").value
        )
        self._min_gps_quality = float(
            self.get_parameter("min_gps_quality_m").value
        )

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # GPS converter
        self._converter = GPSConverter(self, self._tf_buffer)

        # Nav2 client
        self._nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # GPS subscription
        self._last_fix: Optional[NavSatFix] = None
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_cb, 10)

        # Status publisher
        from std_msgs.msg import String

        self._status_pub = self.create_publisher(String, "/gps_navigator/status", 10)

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateToGPS,
            "navigate_to_gps",
            execute_callback=self._execute_cb,
            cancel_callback=self._cancel_cb,
        )

        self._active_goal_handle = None
        self.get_logger().info("GPSNavigatorNode ready (action: navigate_to_gps)")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _gps_cb(self, msg: NavSatFix):
        self._last_fix = msg

    def _publish_status(self, text: str):
        from std_msgs.msg import String

        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("NavigateToGPS goal cancel requested")
        if self._active_goal_handle is not None:
            self._nav2_client.cancel_goal_async(self._active_goal_handle)
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Core execution
    # ------------------------------------------------------------------
    async def _execute_cb(self, goal_handle):
        goal: NavigateToGPS.Goal = goal_handle.request
        self.get_logger().info(
            f"NavigateToGPS received: lat={goal.latitude:.7f}, "
            f"lon={goal.longitude:.7f}, retries={goal.max_retries}"
        )

        # Sanity checks
        if self._last_fix is None:
            self.get_logger().warn("No GPS fix yet - aborting")
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = "No GPS fix"
            result.final_distance_error = math.nan
            return result

        # Initialize reference for converter using current pose
        try:
            now = self.get_clock().now().to_msg()
            # Use latest available transform map -> base_link
            trans = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            ref_x = trans.transform.translation.x
            ref_y = trans.transform.translation.y
            self._converter.set_reference(
                self._last_fix.latitude,
                self._last_fix.longitude,
                ref_x,
                ref_y,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to get TF map->base_link: {exc}")
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = "No TF map->base_link"
            result.final_distance_error = math.nan
            return result

        # Build target pose
        try:
            target_pose = self._converter.gps_to_map_pose(
                goal.latitude, goal.longitude
            )
            target_pose.header.stamp = self.get_clock().now().to_msg()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert GPS to map pose: {exc}")
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = "Conversion error"
            result.final_distance_error = math.nan
            return result

        # Wait for Nav2
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 NavigateToPose action server not available")
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = "Nav2 not available"
            result.final_distance_error = math.nan
            return result

        # Retry loop
        max_tries = max(1, goal.max_retries + 1)
        last_error = ""

        for attempt in range(max_tries):
            self._publish_status(f"NAVIGATING (attempt {attempt + 1}/{max_tries})")

            nav2_goal = NavigateToPose.Goal()
            nav2_goal.pose = target_pose

            send_future = self._nav2_client.send_goal_async(nav2_goal)
            rclpy.task.Future
            await send_future
            nav2_handle = send_future.result()

            if not nav2_handle.accepted:
                last_error = "Nav2 goal rejected"
                self.get_logger().warn(last_error)
                continue

            self._active_goal_handle = nav2_handle

            result_future = nav2_handle.get_result_async()

            # Wait with timeout
            done = await self._wait_with_timeout(result_future, self._goal_timeout)
            if not done:
                self.get_logger().warn("Nav2 goal timeout, cancelling")
                await nav2_handle.cancel_goal_async()
                last_error = "Timeout"
            else:
                nav2_result = result_future.result().result
                status = result_future.result().status
                if status == 4:  # ABORTED
                    last_error = "Nav2 aborted"
                    self.get_logger().warn(last_error)
                elif status == 3:  # SUCCEEDED
                    self.get_logger().info("NavigateToGPS succeeded")
                    goal_handle.succeed()
                    result = NavigateToGPS.Result()
                    result.success = True
                    result.message = "Success"
                    result.final_distance_error = 0.0
                    self._publish_status("SUCCESS")
                    return result
                else:
                    last_error = f"Nav2 returned status {status}"
                    self.get_logger().warn(last_error)

        # All retries failed
        goal_handle.abort()
        self._publish_status("FAILED")
        result = NavigateToGPS.Result()
        result.success = False
        result.message = f"Navigation failed: {last_error}"
        result.final_distance_error = math.nan
        return result

    async def _wait_with_timeout(self, future, timeout_sec: float) -> bool:
        """Await future with timeout."""
        end_time = self.get_clock().now() + Duration(seconds=float(timeout_sec))
        while not future.done():
            if self.get_clock().now() > end_time:
                return False
            await rclpy.task.sleep(0.1)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


