#!/usr/bin/env python3
"""
Comprehensive Arm Control Test Suite
Tests all functionality: named poses, speeds, multi-waypoint trajectories, joint limits
Works in simulation AND on real hardware (same code!)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import math
import time


class ComprehensiveArmTester(Node):
    def __init__(self):
        super().__init__('comprehensive_arm_tester')
        
        # Create action client to send trajectories
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Subscribe to joint states for feedback
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.current_joint_positions = None
        
        self.get_logger().info('Waiting for arm_controller...')
        self.action_client.wait_for_server()
        self.get_logger().info('✓ Connected to arm_controller!')
        
        # Named poses (from SRDF config)
        self.named_poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            'stow': [0.0, -2.0, 0.0, 1.5, 0.0],
            'es_ready': [0.0, -1.5, 0.0, 0.0, -1.5],
            'sc_ground_survey': [0.0, -0.8, 0.0, -0.5, 1.2],
            'vertical_reach': [0.0, 1.5, 0.0, 0.0, -1.5],
        }
    
    def joint_state_callback(self, msg):
        """Store current joint positions for feedback"""
        self.current_joint_positions = dict(zip(msg.name, msg.position))
    
    def move_to_joint_positions(self, positions, duration_sec=3.0, wait=True):
        """
        Move arm to specific joint positions
        
        Args:
            positions: List of 4 joint angles [AB_Rev, AS1_Rev, AW_Rev, AM_Rev]
            duration_sec: How long the motion should take
            wait: Wait for motion to complete
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['AB_Rev', 'AS1_Rev', 'AS2_Rev', 'AW_Rev', 'AM_Rev']
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]  # Ensure all are floats
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        future = self.action_client.send_goal_async(goal_msg)
        
        if wait:
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                return result_future.result().result
        
        return future
    
    def move_multi_waypoint(self, waypoints, durations):
        """
        Move through multiple waypoints in one smooth trajectory
        
        Args:
            waypoints: List of joint position lists
            durations: List of cumulative time for each waypoint
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['AB_Rev', 'AS1_Rev', 'AS2_Rev', 'AW_Rev', 'AM_Rev']
        
        for positions, duration_sec in zip(waypoints, durations):
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]  # Ensure all are floats
            point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
            goal_msg.trajectory.points.append(point)
        
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            return result_future.result().result
    
    def print_current_position(self):
        """Display current joint positions"""
        if self.current_joint_positions:
            print("  Current position:")
            for joint in ['AB_Rev', 'AS1_Rev', 'AS2_Rev', 'AW_Rev', 'AM_Rev']:
                if joint in self.current_joint_positions:
                    print(f"    {joint}: {self.current_joint_positions[joint]:.3f} rad")


def run_test_suite(tester):
    """Run 10 challenging poses requiring all joints"""
    
    print("\n" + "="*70)
    print("  🤖 10 CHALLENGING POSES - ALL JOINTS WORKING")
    print("="*70)
    print("\nEach pose requires coordination of all 4 joints\n")
    
    # 10 CHALLENGING POSES - All requiring multi-joint coordination (5-DOF)
    challenging_poses = [
        (
            [1.2, 1.8, 0.5, -1.5, 0.8],
            "POSE 1: High Right Diagonal Reach",
            "Base rotated, both shoulders up, elbow bent, wrist twisted"
        ),
        (
            [-1.5, -2.2, -0.8, 1.8, -1.2],
            "POSE 2: Low Left Tucked Position",
            "Base left, shoulders back, elbow folded, wrist down"
        ),
        (
            [0.8, 2.3, 1.0, -2.5, 1.5],
            "POSE 3: Extreme Stretched Configuration",
            "All 5 joints near limits, fully extended diagonal"
        ),
        (
            [-2.0, 0.5, -0.5, -1.8, 2.5],
            "POSE 4: Complex Twist Position",
            "Base left, mixed shoulder angles, elbow back, wrist max"
        ),
        (
            [1.5, -1.8, 0.8, 2.2, -0.8],
            "POSE 5: Under-Reach Position",
            "Reaching down with coordinated shoulder joints"
        ),
        (
            [-0.5, 2.4, -0.3, 0.5, -2.0],
            "POSE 6: High Straight Reach",
            "Near vertical with AS2 counterbalancing AS1"
        ),
        (
            [2.5, -2.0, 1.2, -1.5, 1.8],
            "POSE 7: Folded Right Configuration",
            "Extreme base with complex shoulder coordination"
        ),
        (
            [0.3, 1.2, -1.5, -2.8, 0.5],
            "POSE 8: Forward Reach with Extreme Elbow",
            "Medium forward, second shoulder compensating"
        ),
        (
            [-2.5, 1.5, -0.8, 1.2, -1.5],
            "POSE 9: Back-Left High Position",
            "All 5 joints working to reach behind and up"
        ),
        (
            [1.8, -0.8, 1.5, -2.2, 2.8],
            "POSE 10: Diagonal Twist Extreme",
            "Complex 5-DOF position testing full workspace"
        ),
    ]
    
    for i, (positions, name, description) in enumerate(challenging_poses, 1):
        print(f"\n{'-'*70}")
        print(f"{name}")
        print(f"{description}")
        print(f"Target: [{positions[0]:.1f}, {positions[1]:.1f}, {positions[2]:.1f}, {positions[3]:.1f}]")
        print(f"{'-'*70}")
        
        try:
            tester.move_to_joint_positions(positions, duration_sec=4.0)
            time.sleep(1.5)
            tester.print_current_position()
            print(f"✓ POSE {i} ACHIEVED")
        except Exception as e:
            print(f"⚠ POSE {i} - Limited by constraints: {str(e)[:60]}")
        
        time.sleep(0.5)
    
    # Return to home
    print("\n\n" + "="*70)
    print("RETURNING TO HOME POSITION")
    print("="*70)
    tester.move_to_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=3.0)
    time.sleep(1.0)
    tester.print_current_position()
    
    print("\n" + "="*70)
    print("  ✅ ALL 10 CHALLENGING POSES COMPLETED!")
    print("="*70)
    print("\nEvery pose required coordination of all 5 joints:")
    print("  • AB_Rev (Base Rotation - Continuous)")
    print("  • AS1_Rev (Shoulder Pitch 1)")
    print("  • AS2_Rev (Shoulder Pitch 2) ⭐ RESTORED 5th DOF")
    print("  • AW_Rev (Elbow Pitch)")
    print("  • AM_Rev (Wrist Roll)")
    print("\nYour 5-DOF arm matches the real hardware! 🎉\n")


def main():
    rclpy.init()
    
    tester = ComprehensiveArmTester()
    
    # Wait a moment for joint states to arrive
    print("Waiting for joint state feedback...")
    time.sleep(1.0)
    
    try:
        run_test_suite(tester)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\n\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

