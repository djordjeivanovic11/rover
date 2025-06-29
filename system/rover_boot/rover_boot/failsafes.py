#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import BatteryState, Temperature
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
import tf2_ros

class Failsafes:
    """
    Encapsulates all safety and diagnostics checks for the rover:
     • Emergency stop subscriber
     • Diagnostics listener
     • TF staleness monitor
     • Health‐metric subscribers (gap_fps, planner_latency_ms)
     • Battery and temperature monitoring
     • Periodic robot health heartbeat
    On any critical event, publishes zero velocity to /cmd_vel.
    """
    def __init__(self, node: Node, stop_distance: float = 0.6):
        self.node = node
        self.stop_distance = stop_distance
        self.estop_engaged = False

        # Publisher for emergency zero-velocity
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        # Health heartbeat publisher (/robot_health)
        self.health_pub = self.node.create_publisher(Float32, '/robot_health', 1)

        # 1) E-stop subscriber
        self.node.create_subscription(
            Bool, '/e_stop', self._cb_estop, 10)

        # 2) Diagnostics subscriber
        self.node.create_subscription(
            DiagnosticArray, '/diagnostics', self._cb_diag, 10)

        # 3) Health metrics
        self.node.create_subscription(
            Float32, '/gap_fps', self._cb_gap_fps, 10)
        self.node.create_subscription(
            Float32, '/planner_latency_ms', self._cb_planner_lat, 10)

        # 4) Battery and temperature monitoring
        self.node.create_subscription(
            BatteryState, '/battery_state', self._cb_battery, 10)
        self.node.create_subscription(
            Temperature, '/zed2i/temperature/imu', self._cb_temp, 10)

        # 5) TF staleness monitor
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.last_tf = self.node.get_clock().now()
        self.node.create_timer(0.5, self._check_tf)

        # 6) Periodic health heartbeat (1 Hz)
        self.node.create_timer(1.0, self._publish_health)

        # Internal state metrics
        self.last_gap_fps = 0.0
        self.last_planner_lat = 0.0
        self.last_battery = None
        self.last_temp = None

    def _cb_estop(self, msg: Bool):
        if msg.data and not self.estop_engaged:
            self.estop_engaged = True
            self.node.get_logger().warn('🔥 E-STOP engaged!')
            self._publish_zero()

    def _cb_diag(self, msg: DiagnosticArray):
        for status in msg.status:
            if status.level == DiagnosticStatus.ERROR:
                self.node.get_logger().error(
                    f'❌ Diagnostic error: {status.name}')
                # optionally halt on critical errors
                self._publish_zero()

    def _cb_gap_fps(self, msg: Float32):
        self.last_gap_fps = msg.data
        if msg.data < 5.0:
            self.node.get_logger().warn(
                f'⚠️  Gap guidance FPS low: {msg.data:.1f}')

    def _cb_planner_lat(self, msg: Float32):
        self.last_planner_lat = msg.data
        if msg.data > 100.0:
            self.node.get_logger().warn(
                f'⚠️  Planner latency high: {msg.data:.1f} ms')

    def _cb_battery(self, msg: BatteryState):
        self.last_battery = msg.percentage if msg.percentage is not None else None
        if msg.percentage is not None and msg.percentage < 0.2:
            self.node.get_logger().error(
                f'🔋 Battery low: {msg.percentage*100:.0f}%')
            self._publish_zero()

    def _cb_temp(self, msg: Temperature):
        self.last_temp = msg.temperature
        if msg.temperature > 70.0:  # threshold in °C
            self.node.get_logger().error(
                f'🌡️  IMU temp high: {msg.temperature:.1f} °C')
            self._publish_zero()

    def _check_tf(self):
        try:
            now = self.node.get_clock().now().to_msg()
            self.tf_buffer.lookup_transform(
                'map', 'base_link', now)
            self.last_tf = self.node.get_clock().now()
        except Exception:
            if (self.node.get_clock().now() - self.last_tf) > Duration(seconds=1):
                self.node.get_logger().error(
                    '⏰ TF stale >1s – publishing zero velocity')
                self._publish_zero()

    def _publish_zero(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def _publish_health(self):
        # Combine key metrics into a simple health score (0-1)
        score = 1.0
        if self.last_gap_fps < 1.0:
            score -= 0.5
        if self.last_planner_lat > 200.0:
            score -= 0.2
        if self.last_battery is not None and self.last_battery < 0.3:
            score -= 0.3
        self.health_pub.publish(Float32(data=max(0.0, score)))
        self.node.get_logger().debug(f'Health score: {score:.2f}'}
