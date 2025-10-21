#!/usr/bin/env python3
"""
=============================================================================
MISSION ACTION SERVERS FOR URC ROVER ARM
=============================================================================
Provides high-level action servers for URC missions:
- GoToNamedPose: Move to predefined poses
- PickAndPlace: Complete pick and place sequences

These integrate with behavior trees and mission planning.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

# MoveIt
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32

# Custom actions
import sys
import os
sys.path.append(os.path.dirname(__file__))

# Action definitions will be imported after build
# from arm_control.action import GoToNamedPose, PickAndPlace

import time


class ArmActionServers(Node):
    """
    Action servers for high-level arm control in URC missions.
    """
    
    def __init__(self):
        super().__init__('arm_action_servers')
        
        # Initialize MoveIt
        self.get_logger().info('Initializing MoveIt...')
        try:
            self.moveit = MoveItPy(node_name="arm_action_moveit")
            self.arm = self.moveit.get_planning_component("arm")
            self.get_logger().info('MoveIt initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt: {e}')
            raise
        
        # Gripper control publisher
        self.gripper_open_pub = self.create_publisher(Float32, '/gripper/open', 10)
        self.gripper_close_pub = self.create_publisher(Float32, '/gripper/close', 10)
        
        # Import actions dynamically (after messages are built)
        try:
            from arm_control.action import GoToNamedPose, PickAndPlace
            
            # Action servers
            self._goto_action_server = ActionServer(
                self,
                GoToNamedPose,
                '/arm/go_to_named_pose',
                self._execute_goto_named_pose,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback
            )
            
            self._pick_place_action_server = ActionServer(
                self,
                PickAndPlace,
                '/arm/pick_and_place',
                self._execute_pick_and_place,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback
            )
        except ImportError as e:
            self.get_logger().error(f'Failed to import action definitions: {e}')
            self.get_logger().error('Make sure to build the package first: colcon build --packages-select arm_control')
            raise
        
        self.get_logger().info('Arm Action Servers ready')
        self.get_logger().info('  - /arm/go_to_named_pose')
        self.get_logger().info('  - /arm/pick_and_place')
    
    def _goal_callback(self, goal_request):
        """Accept or reject goals"""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle):
        """Accept or reject cancellation requests"""
        self.get_logger().info('Received cancellation request')
        return CancelResponse.ACCEPT
    
    def _execute_goto_named_pose(self, goal_handle):
        """
        Execute GoToNamedPose action.
        Moves arm to a named pose defined in SRDF.
        """
        self.get_logger().info('=== Executing GoToNamedPose ===')
        request = goal_handle.request
        feedback = GoToNamedPose.Feedback()
        result = GoToNamedPose.Result()
        
        pose_name = request.pose_name
        velocity_scaling = request.velocity_scaling if request.velocity_scaling > 0 else 0.1
        
        self.get_logger().info(f'Target pose: {pose_name}')
        self.get_logger().info(f'Velocity scaling: {velocity_scaling}')
        
        try:
            # Set target pose
            feedback.status = f'Planning to {pose_name}...'
            goal_handle.publish_feedback(feedback)
            
            self.arm.set_goal_state(configuration_name=pose_name)
            
            # Plan
            feedback.status = 'Planning trajectory...'
            goal_handle.publish_feedback(feedback)
            
            plan_result = self.arm.plan()
            
            if not plan_result:
                self.get_logger().error(f'Planning failed for pose: {pose_name}')
                result.success = False
                result.message = f'Planning failed for {pose_name}'
                goal_handle.abort()
                return result
            
            # Execute
            feedback.status = 'Executing trajectory...'
            goal_handle.publish_feedback(feedback)
            
            self.moveit.execute(plan_result.trajectory, blocking=True)
            
            # Success
            self.get_logger().info(f'Successfully moved to {pose_name}')
            result.success = True
            result.message = f'Reached {pose_name}'
            goal_handle.succeed()
            
        except Exception as e:
            self.get_logger().error(f'Error executing GoToNamedPose: {e}')
            result.success = False
            result.message = str(e)
            goal_handle.abort()
        
        return result
    
    def _execute_pick_and_place(self, goal_handle):
        """
        Execute PickAndPlace action.
        Complete sequence: approach -> grasp -> lift -> move -> place -> release
        """
        self.get_logger().info('=== Executing PickAndPlace ===')
        request = goal_handle.request
        feedback = PickAndPlace.Feedback()
        result = PickAndPlace.Result()
        
        try:
            # Phase 1: Approach pick position
            feedback.phase = 'approaching'
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Phase 1: Approaching pick position')
            
            # Add small offset above pick pose for approach
            approach_pose = PoseStamped()
            approach_pose.header = request.pick_pose.header
            approach_pose.pose = request.pick_pose.pose
            approach_pose.pose.position.z += 0.1  # 10cm above
            
            if not self._move_to_pose(approach_pose):
                raise Exception("Failed to reach approach position")
            
            # Phase 2: Move to pick position
            feedback.phase = 'picking'
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Phase 2: Moving to pick position')
            
            if not self._move_to_pose(request.pick_pose):
                raise Exception("Failed to reach pick position")
            
            # Phase 3: Close gripper
            self.get_logger().info('Phase 3: Closing gripper')
            self._close_gripper(request.grasp_effort)
            time.sleep(1.0)  # Wait for grasp
            
            # Phase 4: Lift object
            feedback.phase = 'lifting'
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Phase 4: Lifting object')
            
            lift_pose = PoseStamped()
            lift_pose.header = request.pick_pose.header
            lift_pose.pose = request.pick_pose.pose
            lift_pose.pose.position.z += 0.15  # Lift 15cm
            
            if not self._move_to_pose(lift_pose):
                raise Exception("Failed to lift object")
            
            # Phase 5: Move to place approach
            feedback.phase = 'moving'
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Phase 5: Moving to place position')
            
            place_approach_pose = PoseStamped()
            place_approach_pose.header = request.place_pose.header
            place_approach_pose.pose = request.place_pose.pose
            place_approach_pose.pose.position.z += 0.1  # 10cm above
            
            if not self._move_to_pose(place_approach_pose):
                raise Exception("Failed to reach place approach")
            
            # Phase 6: Lower to place position
            feedback.phase = 'placing'
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Phase 6: Placing object')
            
            if not self._move_to_pose(request.place_pose):
                raise Exception("Failed to reach place position")
            
            # Phase 7: Release gripper
            self.get_logger().info('Phase 7: Releasing gripper')
            self._open_gripper()
            time.sleep(0.5)  # Wait for release
            
            # Phase 8: Retract
            feedback.phase = 'retracting'
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Phase 8: Retracting')
            
            retract_pose = PoseStamped()
            retract_pose.header = request.place_pose.header
            retract_pose.pose = request.place_pose.pose
            retract_pose.pose.position.z += 0.1  # Retract 10cm
            
            if not self._move_to_pose(retract_pose):
                self.get_logger().warn("Retract failed, but pick-and-place succeeded")
            
            # Success!
            self.get_logger().info('PickAndPlace completed successfully')
            result.success = True
            result.message = 'Pick and place completed'
            goal_handle.succeed()
            
        except Exception as e:
            self.get_logger().error(f'PickAndPlace failed: {e}')
            result.success = False
            result.message = str(e)
            goal_handle.abort()
        
        return result
    
    def _move_to_pose(self, pose_stamped):
        """Helper: Move to a Cartesian pose"""
        try:
            self.arm.set_goal_state(pose_stamped_msg=pose_stamped)
            plan_result = self.arm.plan()
            
            if plan_result:
                self.moveit.execute(plan_result.trajectory, blocking=True)
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f'Move to pose failed: {e}')
            return False
    
    def _open_gripper(self):
        """Helper: Open gripper"""
        msg = Float32()
        msg.data = 1.0  # Fully open
        self.gripper_open_pub.publish(msg)
    
    def _close_gripper(self, effort=1.0):
        """Helper: Close gripper with specified effort"""
        msg = Float32()
        msg.data = min(1.0, max(0.0, effort))
        self.gripper_close_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        action_servers = ArmActionServers()
        rclpy.spin(action_servers)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Action servers failed: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
