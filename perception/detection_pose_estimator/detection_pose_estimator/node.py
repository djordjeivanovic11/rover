import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf_transformations import quaternion_from_euler

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        # parameters
        self.declare_parameter('detection_topic', '/detected_objects')
        self.declare_parameter('cloud_topic', '/zed2i/point_cloud/cloud_registered')
        self.declare_parameter('output_topic', '/target_pose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('use_closest_point', True)
        self.declare_parameter('confidence_filter', 0.5)

        dt = self.get_parameter('detection_topic').value
        ct = self.get_parameter('cloud_topic').value
        ot = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_conf = self.get_parameter('confidence_filter').value

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        # subs + pub
        self.create_subscription(
            Detection2DArray, dt, self.cb_detections, 10)
        self.create_subscription(
            PointCloud2, ct, self.cb_cloud, 1)
        self.pub = self.create_publisher(PoseStamped, ot, 10)

        # store last detections & cloud
        self.latest_dets = None
        self.latest_cloud = None

    def cb_detections(self, msg: Detection2DArray):
        # keep only high-confidence
        self.latest_dets = [d for d in msg.detections
                             if d.results[0].score >= self.min_conf]

        self.try_publish()

    def cb_cloud(self, msg: PointCloud2):
        self.latest_cloud = msg
        self.try_publish()

    def try_publish(self):
        if not self.latest_dets or not self.latest_cloud:
            return

        # pick the first detection
        det = self.latest_dets[0]
        # image center pixel
        xc = int((det.bbox.center.x))
        yc = int((det.bbox.center.y))

        # get 3D point under that pixel
        for (x, y, z, *_ ) in pc2.read_points(self.latest_cloud,
                                               skip_nans=True,
                                               uvs=[(xc, yc)]):
            if z > 0:
                pt_cam = [x, y, z, 1.0]
                break
        else:
            self.get_logger().warn("No 3D point for detection")
            self.latest_dets = None
            return

        # transform to map frame
        try:
            now = rclpy.time.Time().to_msg()
            t = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.latest_cloud.header.frame_id,
                now)
            # apply transform (4×4) – here we assume identity rotation
            # for brevity; in practice use tf2_geometry_msgs or do proper transform
            x_m = pt_cam[0] + t.transform.translation.x
            y_m = pt_cam[1] + t.transform.translation.y
            z_m = pt_cam[2] + t.transform.translation.z
        except Exception as e:
            self.get_logger().error(f"TF error: {e}")
            return

        # publish PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x_m
        pose.pose.position.y = y_m
        pose.pose.position.z = z_m
        # facing forward (no yaw)
        q = quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pub.publish(pose)
        # clear buffered data
        self.latest_dets = None
        self.latest_cloud = None

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    rclpy.shutdown()
