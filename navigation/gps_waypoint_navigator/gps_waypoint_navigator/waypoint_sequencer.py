#!/usr/bin/env python3
"""
FollowGPSWaypoints action server.

Accepts a sequence of GPS waypoints and navigates to each one in order.
"""

import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node

from urc_msgs.action import FollowGPSWaypoints, NavigateToGPS  # type: ignore


class WaypointSequencer(Node):
    def __init__(self):
        super().__init__("waypoint_sequencer")

        # Client for single waypoint navigation
        self._nav_client = ActionClient(self, NavigateToGPS, "navigate_to_gps")

        # Action server for sequences
        self._action_server = ActionServer(
            self,
            FollowGPSWaypoints,
            "follow_gps_waypoints",
            execute_callback=self._execute_cb,
            cancel_callback=self._cancel_cb,
        )

        self._active_nav_handle = None
        self.get_logger().info("WaypointSequencer ready (action: follow_gps_waypoints)")

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("FollowGPSWaypoints goal cancel requested")
        if self._active_nav_handle is not None:
            self.get_logger().info("Cancelling current navigation goal")
            self._nav_client.cancel_goal_async(self._active_nav_handle)
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        goal: FollowGPSWaypoints.Goal = goal_handle.request
        self.get_logger().info(
            f"FollowGPSWaypoints received: {len(goal.latitudes)} waypoints, "
            f"skip_on_failure={goal.skip_on_failure}, "
            f"abort_on_first_failure={goal.abort_on_first_failure}"
        )

        # Validate input
        n = len(goal.latitudes)
        if n == 0:
            self.get_logger().warn("Empty waypoint list")
            goal_handle.abort()
            result = FollowGPSWaypoints.Result()
            result.success = False
            result.message = "Empty waypoint list"
            result.waypoints_completed = 0
            return result

        if len(goal.longitudes) != n:
            self.get_logger().error("Latitude and longitude arrays have different lengths")
            goal_handle.abort()
            result = FollowGPSWaypoints.Result()
            result.success = False
            result.message = "Mismatched lat/lon arrays"
            result.waypoints_completed = 0
            return result

        # Names can be optional
        names = goal.waypoint_names if len(goal.waypoint_names) == n else [f"wp_{i}" for i in range(n)]

        # Wait for NavigateToGPS server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToGPS action server not available")
            goal_handle.abort()
            result = FollowGPSWaypoints.Result()
            result.success = False
            result.message = "NavigateToGPS server not available"
            result.waypoints_completed = 0
            return result

        # Navigate to each waypoint
        waypoints_completed = 0
        for i in range(n):
            lat = goal.latitudes[i]
            lon = goal.longitudes[i]
            name = names[i]

            self.get_logger().info(
                f"Navigating to waypoint {i+1}/{n}: '{name}' (lat={lat:.7f}, lon={lon:.7f})"
            )

            # Publish feedback
            feedback = FollowGPSWaypoints.Feedback()
            feedback.status = "NAVIGATING"
            feedback.current_index = i
            feedback.current_name = name
            feedback.distance_remaining = 0.0
            goal_handle.publish_feedback(feedback)

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
                self.get_logger().error(f"NavigateToGPS goal rejected for waypoint '{name}'")
                if goal.abort_on_first_failure:
                    goal_handle.abort()
                    result = FollowGPSWaypoints.Result()
                    result.success = False
                    result.message = f"Goal rejected at waypoint '{name}'"
                    result.waypoints_completed = waypoints_completed
                    return result
                elif not goal.skip_on_failure:
                    # If not skipping and not aborting, still treat as failure
                    goal_handle.abort()
                    result = FollowGPSWaypoints.Result()
                    result.success = False
                    result.message = f"Goal rejected at waypoint '{name}'"
                    result.waypoints_completed = waypoints_completed
                    return result
                else:
                    self.get_logger().warn(f"Skipping waypoint '{name}' (goal rejected)")
                    continue

            self._active_nav_handle = nav_handle

            # Wait for result
            result_future = nav_handle.get_result_async()
            await result_future

            nav_result = result_future.result().result
            status = result_future.result().status

            if status == 3:  # SUCCEEDED
                self.get_logger().info(f"Successfully reached waypoint '{name}'")
                waypoints_completed += 1
            elif status == 2:  # CANCELED
                self.get_logger().warn(f"Navigation to '{name}' was cancelled")
                goal_handle.canceled()
                result = FollowGPSWaypoints.Result()
                result.success = False
                result.message = f"Cancelled at waypoint '{name}'"
                result.waypoints_completed = waypoints_completed
                return result
            else:  # ABORTED or other failure
                self.get_logger().error(
                    f"Navigation to '{name}' failed: {nav_result.message}"
                )
                if goal.abort_on_first_failure:
                    goal_handle.abort()
                    result = FollowGPSWaypoints.Result()
                    result.success = False
                    result.message = f"Failed at waypoint '{name}': {nav_result.message}"
                    result.waypoints_completed = waypoints_completed
                    return result
                elif not goal.skip_on_failure:
                    goal_handle.abort()
                    result = FollowGPSWaypoints.Result()
                    result.success = False
                    result.message = f"Failed at waypoint '{name}': {nav_result.message}"
                    result.waypoints_completed = waypoints_completed
                    return result
                else:
                    self.get_logger().warn(f"Skipping failed waypoint '{name}'")

        # All waypoints processed
        success = waypoints_completed == n
        if success:
            self.get_logger().info(f"All {n} waypoints reached successfully")
            goal_handle.succeed()
        else:
            self.get_logger().warn(
                f"Completed {waypoints_completed}/{n} waypoints (some skipped)"
            )
            goal_handle.succeed()  # Still succeed if we completed with skip_on_failure

        result = FollowGPSWaypoints.Result()
        result.success = success
        result.message = f"Completed {waypoints_completed}/{n} waypoints"
        result.waypoints_completed = waypoints_completed
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

