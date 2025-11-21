#!/usr/bin/env python3
"""
GoToNamedWaypoint action server.

Looks up named waypoints from a YAML config and calls NavigateToGPS.
"""

from pathlib import Path

import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from urc_msgs.action import GoToNamedWaypoint, NavigateToGPS  # type: ignore

from .waypoint_manager import WaypointManager


class NamedWaypointNode(Node):
    def __init__(self):
        super().__init__("named_waypoint_node")

        # Parameter for waypoint config file
        self.declare_parameter("waypoint_config", "")
        config_path = self.get_parameter("waypoint_config").value

        if not config_path:
            # Default to package share directory
            pkg_share = get_package_share_directory("gps_waypoint_navigator")
            config_path = str(Path(pkg_share) / "config" / "waypoints.yaml")

        self.get_logger().info(f"Loading waypoints from: {config_path}")
        self._waypoint_manager = WaypointManager(Path(config_path))

        # Client for NavigateToGPS
        self._nav_client = ActionClient(self, NavigateToGPS, "navigate_to_gps")

        # Action server
        self._action_server = ActionServer(
            self,
            GoToNamedWaypoint,
            "go_to_named_waypoint",
            execute_callback=self._execute_cb,
            cancel_callback=self._cancel_cb,
        )

        self._active_nav_handle = None
        self.get_logger().info("NamedWaypointNode ready (action: go_to_named_waypoint)")

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("GoToNamedWaypoint goal cancel requested")
        if self._active_nav_handle is not None:
            self.get_logger().info("Cancelling current navigation goal")
            self._nav_client.cancel_goal_async(self._active_nav_handle)
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        goal: GoToNamedWaypoint.Goal = goal_handle.request
        name = goal.waypoint_name

        self.get_logger().info(
            f"GoToNamedWaypoint received: name='{name}', retries={goal.max_retries}"
        )

        # Lookup waypoint
        try:
            lat, lon = self._waypoint_manager.lookup(name)
            self.get_logger().info(
                f"Waypoint '{name}' found: lat={lat:.7f}, lon={lon:.7f}"
            )
        except KeyError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            result = GoToNamedWaypoint.Result()
            result.success = False
            result.message = str(e)
            result.final_distance_error = float("nan")
            return result

        # Wait for NavigateToGPS server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToGPS action server not available")
            goal_handle.abort()
            result = GoToNamedWaypoint.Result()
            result.success = False
            result.message = "NavigateToGPS server not available"
            result.final_distance_error = float("nan")
            return result

        # Call NavigateToGPS
        nav_goal = NavigateToGPS.Goal()
        nav_goal.latitude = lat
        nav_goal.longitude = lon
        nav_goal.waypoint_name = name
        nav_goal.max_retries = goal.max_retries

        send_future = self._nav_client.send_goal_async(nav_goal)
        await send_future

        nav_handle = send_future.result()
        if not nav_handle.accepted:
            self.get_logger().error(
                f"NavigateToGPS goal rejected for waypoint '{name}'")
            goal_handle.abort()
            result = GoToNamedWaypoint.Result()
            result.success = False
            result.message = "NavigateToGPS goal rejected"
            result.final_distance_error = float("nan")
            return result

        self._active_nav_handle = nav_handle

        # Wait for result
        result_future = nav_handle.get_result_async()
        await result_future

        nav_result = result_future.result().result
        status = result_future.result().status

        # Map result back
        result = GoToNamedWaypoint.Result()
        result.success = nav_result.success
        result.message = nav_result.message
        result.final_distance_error = nav_result.final_distance_error

        if status == 3:  # SUCCEEDED
            self.get_logger().info(f"Successfully reached waypoint '{name}'")
            goal_handle.succeed()
        elif status == 2:  # CANCELED
            self.get_logger().warn(f"Navigation to '{name}' was cancelled")
            goal_handle.canceled()
        else:  # ABORTED or other failure
            self.get_logger().error(
                f"Navigation to '{name}' failed: {nav_result.message}")
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = NamedWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
