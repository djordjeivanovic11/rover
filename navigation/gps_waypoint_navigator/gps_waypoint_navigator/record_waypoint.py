#!/usr/bin/env python3
"""
Simple CLI tool to record the current GPS position as a named waypoint.

Usage:
  ros2 run gps_waypoint_navigator record_waypoint --name my_point
"""

import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import yaml


class WaypointRecorder(Node):
    def __init__(self, name: str, yaml_path: Path):
        super().__init__("waypoint_recorder")
        self._target_name = name
        self._yaml_path = yaml_path
        self._got_fix = False
        self._lat = 0.0
        self._lon = 0.0

        self._sub = self.create_subscription(
            NavSatFix, "/gps/fix", self._gps_cb, 10
        )

        self.get_logger().info(
            f"Recording waypoint '{self._target_name}' to {self._yaml_path}"
        )

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warn("No GPS fix yet, waiting...")
            return

        self._lat = msg.latitude
        self._lon = msg.longitude
        self._got_fix = True

        self.get_logger().info(
            f"Got GPS fix: lat={self._lat:.7f}, lon={self._lon:.7f}"
        )

        self._save_and_exit()

    def _save_and_exit(self):
        # Load existing YAML if present
        data = {}
        if self._yaml_path.exists():
            with self._yaml_path.open("r") as f:
                data = yaml.safe_load(f) or {}

        waypoints = data.get("waypoints", {}) or {}
        waypoints[self._target_name] = {
            "latitude": float(self._lat),
            "longitude": float(self._lon),
        }
        data["waypoints"] = waypoints

        self._yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with self._yaml_path.open("w") as f:
            yaml.safe_load
            yaml.dump(data, f)

        self.get_logger().info("Waypoint saved. Shutting down.")
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--name", required=True, help="Name of waypoint to record"
    )
    parser.add_argument(
        "--file",
        default=str(
            Path.home()
            / "workspaces"
            / "rover"
            / "src"
            / "navigation"
            / "gps_waypoint_navigator"
            / "config"
            / "waypoints.yaml"
        ),
        help="YAML file to store waypoints",
    )
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = WaypointRecorder(
        cli_args.name,
        Path(cli_args.file),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
