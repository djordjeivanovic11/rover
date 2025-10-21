#!/usr/bin/env python3
"""
=============================================================================
MOVEIT ARM CONTROLLER
=============================================================================
High-level controller for the rover arm using MoveIt2.
Provides services and action servers for:
- Moving to Cartesian poses (replaces old IK service)
- Moving to named poses
- Executing trajectories
- Getting current end effector pose

This replaces the old custom IK service with MoveIt's powerful planning.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

# MoveIt imports
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# Standard ROS messages
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# Custom service (compatible with old system)
try:
    from interfaces.srv import IK as IKService
    LEGACY_SERVICE_AVAILABLE = True
except ImportError:
    LEGACY_SERVICE_AVAILABLE = False
    print("Warning: Legacy IK service interface not available")

import numpy as np
from scipy.spatial.transform import Rotation


class MoveItArmController(Node):
    """
    MoveIt-based arm controller providing high-level motion control.
    """
    
    def __init__(self):
        super().__init__('moveit_arm_controller')
        
        # Initialize MoveItPy
        self.get_logger().info('Initializing MoveIt...')
        self.moveit = MoveItPy(node_name="moveit_py_node")
        self.arm = self.moveit.get_planning_component("arm")
        self.robot = self.moveit.get_robot_model()
        
        # Get planning scene and robot state
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        
        # Current robot state
        self.current_joint_positions = np.zeros(4)  # 4 DOF arm
        self.current_ee_pose = None
        
        # Joint names
        self.joint_names = ['AB_Rev', 'AS1_Rev', 'AW_Rev', 'AM_Rev']
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # Publishers
        self.ee_pose_pub = self.create_publisher(
            PoseStamped,
            '/arm/end_effector_pose',
            10
        )
        
        self.ee_position_pub = self.create_publisher(
            Float32MultiArray,
            '/current_cart',  # Compatible with old system
            10
        )
        
        self.ee_rotation_pub = self.create_publisher(
            Float32MultiArray,
            '/current_rot',  # Compatible with old system
            10
        )
        
        # Services
        self.move_to_pose_service = self.create_service(
            GetPositionIK,
            '/arm/move_to_pose',
            self._move_to_pose_callback
        )
        
        # Legacy service for backward compatibility
        if LEGACY_SERVICE_AVAILABLE:
            self.legacy_ik_service = self.create_service(
                IKService,
                '/arm_position',  # Same topic as old system
                self._legacy_ik_callback
            )
            self.get_logger().info('Legacy IK service created for backward compatibility')
        
        # Timer for publishing current pose
        self.pose_pub_timer = self.create_timer(0.1, self._publish_current_pose)
        
        self.get_logger().info('MoveIt Arm Controller initialized successfully')
    
    def _joint_state_callback(self, msg):
        """Update current joint positions from joint state messages"""
        try:
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_joint_positions[i] = msg.position[idx]
        except Exception as e:
            self.get_logger().error(f'Error processing joint states: {e}')
    
    def _publish_current_pose(self):
        """Publish current end effector pose"""
        try:
            # Get current robot state
            with self.planning_scene_monitor.read_write() as scene:
                robot_state = scene.current_state
                
                # Get end effector pose
                ee_link = robot_state.get_global_link_transform('tool_flange')
                
                # Extract position and orientation
                position = ee_link.translation()
                rotation_matrix = ee_link.rotation()
                
                # Convert to Euler angles (XYZ convention)
                rotation = Rotation.from_matrix(rotation_matrix)
                euler = rotation.as_euler('xyz')
                
                # Publish pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'base_link'
                pose_msg.pose.position.x = position[0]
                pose_msg.pose.position.y = position[1]
                pose_msg.pose.position.z = position[2]
                
                quat = rotation.as_quat()  # Returns [x, y, z, w]
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                
                self.ee_pose_pub.publish(pose_msg)
                
                # Publish in old system format for compatibility
                pos_msg = Float32MultiArray()
                pos_msg.data = [float(position[0]), float(position[1]), float(position[2])]
                self.ee_position_pub.publish(pos_msg)
                
                rot_msg = Float32MultiArray()
                rot_msg.data = [float(euler[0]), float(euler[1]), float(euler[2])]
                self.ee_rotation_pub.publish(rot_msg)
                
        except Exception as e:
            self.get_logger().debug(f'Error publishing current pose: {e}')
    
    def _move_to_pose_callback(self, request, response):
        """
        Service callback to move arm to a Cartesian pose.
        Uses MoveIt to plan and execute the motion.
        """
        try:
            self.get_logger().info(f'Moving to pose: {request.ik_request.pose_stamped.pose.position}')
            
            # Set target pose
            self.arm.set_goal_state(pose_stamped_msg=request.ik_request.pose_stamped)
            
            # Plan trajectory
            self.get_logger().info('Planning trajectory...')
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info('Plan successful, executing...')
                # Execute trajectory
                self.moveit.execute(plan_result.trajectory)
                response.error_code.val = MoveItErrorCodes.SUCCESS
            else:
                self.get_logger().error('Planning failed')
                response.error_code.val = MoveItErrorCodes.PLANNING_FAILED
            
        except Exception as e:
            self.get_logger().error(f'Error in move_to_pose: {e}')
            response.error_code.val = MoveItErrorCodes.FAILURE
        
        return response
    
    def _legacy_ik_callback(self, request, response):
        """
        Legacy IK service callback for backward compatibility with old system.
        Converts old service format to MoveIt planning.
        """
        try:
            self.get_logger().info(f'Legacy IK request: pos=[{request.x}, {request.y}, {request.z}], '
                                  f'rot=[{request.rx}, {request.ry}, {request.rz}]')
            
            # Create target pose from request
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = float(request.x)
            target_pose.pose.position.y = float(request.y)
            target_pose.pose.position.z = float(request.z)
            
            # Convert Euler angles to quaternion
            rotation = Rotation.from_euler('xyz', [request.rx, request.ry, request.rz])
            quat = rotation.as_quat()  # Returns [x, y, z, w]
            target_pose.pose.orientation.x = quat[0]
            target_pose.pose.orientation.y = quat[1]
            target_pose.pose.orientation.z = quat[2]
            target_pose.pose.orientation.w = quat[3]
            
            # Set target pose for planning
            self.arm.set_goal_state(pose_stamped_msg=target_pose)
            
            # Plan trajectory
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info('Planning successful')
                
                # Get final joint positions from trajectory
                trajectory = plan_result.trajectory
                if trajectory and len(trajectory.joint_trajectory.points) > 0:
                    final_point = trajectory.joint_trajectory.points[-1]
                    joint_positions = final_point.positions
                    
                    # Fill response (matching old interface)
                    response.valid = 0  # 0 means success in old system
                    if len(joint_positions) >= 4:
                        response.out.joint1 = float(joint_positions[0])  # AB_Rev
                        response.out.joint2 = float(joint_positions[1])  # AS1_Rev
                        response.out.joint3 = 0.0  # Not used in our 4-DOF arm
                        response.out.joint4 = float(joint_positions[2])  # AW_Rev
                        response.out.joint5 = float(joint_positions[3])  # AM_Rev
                    
                    # Execute the planned trajectory
                    self.moveit.execute(trajectory)
                else:
                    response.valid = 1  # Planning succeeded but no trajectory
            else:
                self.get_logger().error('Planning failed')
                response.valid = 1  # 1 means failure in old system
            
        except Exception as e:
            self.get_logger().error(f'Error in legacy IK service: {e}')
            response.valid = 1
        
        return response
    
    def move_to_named_pose(self, pose_name):
        """
        Move to a named pose defined in SRDF.
        
        Args:
            pose_name: Name of the pose (e.g., 'home', 'stow', 'es_ready')
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.get_logger().info(f'Moving to named pose: {pose_name}')
            
            # Set named target
            self.arm.set_goal_state(configuration_name=pose_name)
            
            # Plan
            plan_result = self.arm.plan()
            
            if plan_result:
                # Execute
                self.moveit.execute(plan_result.trajectory)
                self.get_logger().info(f'Successfully moved to {pose_name}')
                return True
            else:
                self.get_logger().error(f'Failed to plan to {pose_name}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error moving to named pose: {e}')
            return False
    
    def get_current_joint_positions(self):
        """Get current joint positions"""
        return self.current_joint_positions.copy()
    
    def get_current_ee_pose(self):
        """Get current end effector pose"""
        return self.current_ee_pose


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = MoveItArmController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Controller error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

