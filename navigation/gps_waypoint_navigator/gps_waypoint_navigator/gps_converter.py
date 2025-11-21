import math
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer


class GPSConverter:
    """Utility to convert GPS (lat, lon) targets to poses in the map frame.

    This implementation uses a local tangent-plane approximation:
    - Treat differences in latitude/longitude as linear over small areas
    - Convert deltas to meters (north/east)
    - Add those deltas to the robot's current pose in the map frame
    """

    def __init__(self, node: Node, tf_buffer: Buffer):
        self._node = node
        self._tf_buffer = tf_buffer

        # Reference GPS + map pose (set on first use)
        self._ref_lat: Optional[float] = None
        self._ref_lon: Optional[float] = None
        self._ref_x: Optional[float] = None
        self._ref_y: Optional[float] = None

    def set_reference(self, lat: float, lon: float, x: float, y: float) -> None:
        """Set the reference GPS and map coordinates.

        The converter will compute target positions as offsets from this reference.
        """
        self._ref_lat = lat
        self._ref_lon = lon
        self._ref_x = x
        self._ref_y = y
        self._node.get_logger().info(
            f"GPSConverter reference set: lat={lat:.7f}, lon={lon:.7f}, "
            f"map=({x:.3f}, {y:.3f})"
        )

    def has_reference(self) -> bool:
        return (
            self._ref_lat is not None
            and self._ref_lon is not None
            and self._ref_x is not None
            and self._ref_y is not None
        )

    def _latlon_to_offsets(self, lat: float, lon: float) -> Tuple[float, float]:
        """Approximate conversion from lat/lon to local ENU offsets (meters).

        Uses a simple equirectangular approximation suitable for small distances.
        """
        if self._ref_lat is None or self._ref_lon is None:
            raise RuntimeError("GPSConverter reference not set")

        # Average latitude in radians
        lat_rad = math.radians((lat + self._ref_lat) * 0.5)

        # Rough meters per degree
        m_per_deg_lat = 111_132.92 - 559.82 * math.cos(2 * lat_rad) + \
            1.175 * math.cos(4 * lat_rad)
        m_per_deg_lon = 111_412.84 * math.cos(lat_rad) - \
            93.5 * math.cos(3 * lat_rad)

        d_lat = lat - self._ref_lat
        d_lon = lon - self._ref_lon

        # North (y), East (x)
        dy = d_lat * m_per_deg_lat
        dx = d_lon * m_per_deg_lon

        return dx, dy

    def gps_to_map_pose(self, lat: float, lon: float, frame_id: str = "map") -> PoseStamped:
        """Convert a GPS coordinate to a PoseStamped in the map frame.

        Assumes that:
        - A reference has been set using the robot's current GPS + map pose
        - Map axes are approximately aligned with ENU (East/North)
        """
        if not self.has_reference():
            raise RuntimeError("GPSConverter reference not initialized")

        dx, dy = self._latlon_to_offsets(lat, lon)

        x = self._ref_x + dx
        y = self._ref_y + dy

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        # NOTE: timestamp will be filled by caller
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # Orientation: default to facing forward; Nav2 controller will handle heading
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        return pose
