"""Simple mission executor for sequential GPS waypoint navigation."""

from __future__ import annotations

from typing import List

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from urc_msgs.action import GoToNamedWaypoint  # type: ignore


class SimpleMissionExecutor(Node):
    """Execute a fixed sequence of named waypoints using GoToNamedWaypoint."""

    def __init__(self) -> None:
        super().__init__("simple_mission_executor")

        default_sequence = [
            "start_zone",
            "science_1",
            "science_2",
            "return_point",
        ]

        self.declare_parameter("waypoint_sequence", default_sequence)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("continue_on_failure", False)

        self._waypoints = self._get_waypoint_sequence()
        raw_max_retries = self.get_parameter("max_retries").value
        raw_continue = self.get_parameter("continue_on_failure").value

        self._max_retries = int(raw_max_retries)
        self._continue_on_failure = self._as_bool(raw_continue)

        self._action_client = ActionClient(
            self,
            GoToNamedWaypoint,
            "/go_to_named_waypoint",
        )

    def _get_waypoint_sequence(self) -> List[str]:
        raw_value = self.get_parameter("waypoint_sequence").value

        if isinstance(raw_value, str):
            candidates = raw_value.split(",")
        elif isinstance(raw_value, (list, tuple)):
            candidates = raw_value
        else:
            candidates = []

        return [str(entry).strip() for entry in candidates if str(entry).strip()]

    def run_mission(self) -> bool:
        if not self._waypoints:
            self.get_logger().warn("No waypoints provided; mission will not run.")
            return False

        self.get_logger().info(
            "Starting mission with waypoints: %s", ", ".join(self._waypoints)
        )

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "GoToNamedWaypoint action server not available."
            )
            return False

        successes = 0
        for waypoint in self._waypoints:
            if self._navigate_to_waypoint(waypoint):
                successes += 1
            elif not self._continue_on_failure:
                self.get_logger().error(
                    "Mission aborted after failing to reach '%s'.",
                    waypoint,
                )
                break

        self.get_logger().info(
            "Mission complete: %d/%d waypoints reached.",
            successes,
            len(self._waypoints),
        )
        return successes == len(self._waypoints)

    @staticmethod
    def _as_bool(value: object) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"true", "1", "yes", "on"}
        return bool(value)

    def _navigate_to_waypoint(self, waypoint: str) -> bool:
        goal = GoToNamedWaypoint.Goal()
        goal.waypoint_name = waypoint
        goal.max_retries = self._max_retries

        self.get_logger().info("Navigating to waypoint '%s'", waypoint)

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected for waypoint '%s'", waypoint)
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_info = result_future.result()

        if result_info is None:
            self.get_logger().error(
                "No result returned for waypoint '%s'", waypoint
            )
            return False

        status = result_info.status
        result = result_info.result

        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(
                "Reached waypoint '%s' (error %.2f m)",
                waypoint,
                result.final_distance_error,
            )
            return True

        self.get_logger().warn(
            "Failed to reach '%s': %s",
            waypoint,
            result.message,
        )
        return False


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimpleMissionExecutor()

    try:
        mission_success = node.run_mission()
        if mission_success:
            node.get_logger().info("Mission finished successfully.")
        else:
            node.get_logger().error("Mission finished with failures.")
    except KeyboardInterrupt:
        node.get_logger().info("Mission interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
