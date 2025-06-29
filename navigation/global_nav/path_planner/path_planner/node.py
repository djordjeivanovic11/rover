import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

from .grid import OccupancyGridMap
from .d_star_lite import DStarLite


class PathPlannerNode(Node):
    """
    ROS2 node that performs global planning on a 2D occupancy grid
    using incremental D* Lite.
    Subscribes:
      • /map   (nav_msgs/OccupancyGrid)
      • /goal  (geometry_msgs/PoseStamped)
    Publishes:
      • /planned_path (nav_msgs/Path)
    """

    def __init__(self):
        super().__init__('path_planner')

        # Declare and read parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('planner.connectivity', '8N'),
                ('planner.obstacle_threshold', 50),
            ])
        self.connectivity   = self.get_parameter('planner.connectivity').value
        self.obstacle_thresh = self.get_parameter('planner.obstacle_threshold').value

        # TF buffer & listener
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # Storage for latest map & goal
        self.map_msg = None
        self.goal_msg = None

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.create_subscription(PoseStamped,  '/goal', self.goal_callback, 1)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 1)

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.try_plan()

    def goal_callback(self, msg: PoseStamped):
        self.goal_msg = msg
        self.try_plan()

    def try_plan(self):
        # Only proceed if we have both map and goal
        if self.map_msg is None or self.goal_msg is None:
            return

        # Build our OccupancyGridMap
        info = self.map_msg.info
        grid = OccupancyGridMap(
            width=info.width,
            height=info.height,
            connectivity=self.connectivity)
        grid.set_from_ros(self.map_msg.data, self.obstacle_thresh)

        # Determine start cell by reading base_link→map transform
        map_frame = self.map_msg.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame,
                'base_link',
                Time())
        except TransformException as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            return

        start_x = tf.transform.translation.x
        start_y = tf.transform.translation.y

        # Convert world coords to grid indices
        origin = info.origin.position
        res    = info.resolution
        start_row = int((start_y - origin.y) / res)
        start_col = int((start_x - origin.x) / res)

        # Determine goal cell (transform if not in map frame)
        goal_in_map = self.goal_msg
        if self.goal_msg.header.frame_id != map_frame:
            try:
                # transform goal pose into map frame
                goal_in_map = do_transform_pose(self.goal_msg,
                                                self.tf_buffer.lookup_transform(
                                                    map_frame,
                                                    self.goal_msg.header.frame_id,
                                                    Time()))
            except TransformException as e:
                self.get_logger().warn(f'Could not transform goal: {e}')
                return

        gx = goal_in_map.pose.position.x
        gy = goal_in_map.pose.position.y
        goal_row = int((gy - origin.y) / res)
        goal_col = int((gx - origin.x) / res)

        start = (start_row, start_col)
        goal  = (goal_row,  goal_col)

        # Instantiate D* Lite and compute path
        planner = DStarLite(grid, start, goal)
        planner.compute_shortest_path()
        cell_path = planner.get_path()

        # Convert to nav_msgs/Path
        path_msg = Path()
        path_msg.header = Header(
            frame_id=map_frame,
            stamp=self.get_clock().now().to_msg())

        for (r, c) in cell_path:
            ps = PoseStamped()
            ps.header = path_msg.header
            # center of cell → world coords
            ps.pose.position.x = origin.x + (c + 0.5) * res
            ps.pose.position.y = origin.y + (r + 0.5) * res
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        # Publish the planned path
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
