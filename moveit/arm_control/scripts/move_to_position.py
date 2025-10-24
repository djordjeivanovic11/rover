#!/usr/bin/env python3
"""
=============================================================================
SIMPLE ARM POSITION CONTROLLER
=============================================================================
Inspired by old arm dcode - waits for position input and moves arm there.

Usage:
    python3 move_to_position.py

Then enter positions when prompted:
    x y z [rx ry rz]
    
Example:
    0.5 0.2 0.3 0 3.14 0
    
Or use via ROS service for automated control.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
import sys
import numpy as np
from scipy.spatial.transform import Rotation

# Custom service for position commands
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger


class SimpleArmController(Node):
    """
    Simple arm controller that accepts position commands and moves there.
    Similar to old ros_arm_interface.py but using MoveIt.
    """
    
    def __init__(self):
        super().__init__('simple_arm_controller')
        
        self.get_logger().info('╔════════════════════════════════════════╗')
        self.get_logger().info('║   SIMPLE ARM POSITION CONTROLLER      ║')
        self.get_logger().info('║   Enter coordinates to move arm       ║')
        self.get_logger().info('╚════════════════════════════════════════╝')
        
        # Initialize MoveIt
        self.get_logger().info('Initializing MoveIt...')
        try:
            self.moveit = MoveItPy(node_name="simple_arm_moveit")
            self.arm = self.moveit.get_planning_component("arm")
            self.get_logger().info('✓ MoveIt ready!')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to initialize MoveIt: {e}')
            raise
        
        # Service for programmatic control (like old IK service)
        self.position_service = self.create_service(
            Trigger,  # We'll use a simple trigger service, can upgrade to custom later
            'arm/move_to_position',
            self._handle_position_service
        )
        
        self.get_logger().info('✓ Ready for commands!')
        self.get_logger().info('')
        self.get_logger().info('USAGE:')
        self.get_logger().info('  Interactive: Enter "x y z [rx ry rz]" at prompt')
        self.get_logger().info('  Service: ros2 service call /arm/move_to_position std_srvs/srv/Trigger')
        self.get_logger().info('')
        self.get_logger().info('Format: x y z [rx ry rz]')
        self.get_logger().info('  x, y, z: Position in meters')
        self.get_logger().info('  rx, ry, rz: Rotation in radians (optional, defaults to 0 pi 0)')
        self.get_logger().info('')
        self.get_logger().info('Examples:')
        self.get_logger().info('  0.5 0.2 0.3          (position only)')
        self.get_logger().info('  0.5 0.2 0.3 0 3.14 0 (position + rotation)')
        self.get_logger().info('  home                 (named pose)')
        self.get_logger().info('  quit                 (exit)')
        self.get_logger().info('')
        
        # Current position tracking (like old arm code)
        self.current_position = None
        
    def _handle_position_service(self, request, response):
        """Service handler for programmatic control"""
        self.get_logger().info('Position service called')
        response.success = True
        response.message = 'Service interface ready - use interactive mode or custom service'
        return response
    
    def move_to_position(self, x, y, z, rx=0.0, ry=np.pi, rz=0.0, velocity_scaling=0.1):
        """
        Move arm to specified position with orientation.
        Similar to old IK + FK workflow but using MoveIt.
        
        Args:
            x, y, z: Position in meters (in base_link frame)
            rx, ry, rz: Rotation in radians (Euler XYZ)
            velocity_scaling: Speed scaling factor (0.0 to 1.0)
        
        Returns:
            bool: Success status
        """
        self.get_logger().info('┌─────────────────────────────────────┐')
        self.get_logger().info('│ PLANNING MOVEMENT                   │')
        self.get_logger().info('└─────────────────────────────────────┘')
        self.get_logger().info(f'Target position: [{x:.3f}, {y:.3f}, {z:.3f}]')
        self.get_logger().info(f'Target rotation: [{rx:.3f}, {ry:.3f}, {rz:.3f}] rad')
        
        try:
            # Create target pose (like old arm.py FK output format)
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Set position
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            
            # Convert Euler angles to quaternion (like old arm.py)
            rotation = Rotation.from_euler('xyz', [rx, ry, rz], degrees=False)
            quat = rotation.as_quat()  # [x, y, z, w]
            
            target_pose.pose.orientation.x = quat[0]
            target_pose.pose.orientation.y = quat[1]
            target_pose.pose.orientation.z = quat[2]
            target_pose.pose.orientation.w = quat[3]
            
            # Set goal state (MoveIt IK solver will handle this)
            self.get_logger().info('Running IK...')
            self.arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="tool0")
            
            # Plan trajectory
            self.get_logger().info('Planning trajectory...')
            plan_result = self.arm.plan()
            
            if not plan_result:
                self.get_logger().error('✗ Planning FAILED!')
                self.get_logger().error('  Target may be unreachable or in collision')
                self.get_logger().error('  Check joint limits and workspace bounds')
                return False
            
            self.get_logger().info('✓ Planning successful!')
            
            # Execute trajectory (like old motor control)
            self.get_logger().info('Executing movement...')
            self.moveit.execute(plan_result.trajectory, blocking=True)
            
            self.get_logger().info('✓ Movement complete!')
            self.get_logger().info('')
            
            # Store current position
            self.current_position = (x, y, z, rx, ry, rz)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'✗ Movement failed: {e}')
            return False
    
    def move_to_named_pose(self, pose_name, velocity_scaling=0.1):
        """
        Move to a named pose (from SRDF).
        
        Args:
            pose_name: Name of pose (e.g., 'home', 'stow', 'es_ready')
            velocity_scaling: Speed scaling factor
        
        Returns:
            bool: Success status
        """
        self.get_logger().info(f'Moving to named pose: {pose_name}')
        
        try:
            # Set goal to named configuration
            self.arm.set_goal_state(configuration_name=pose_name)
            
            # Plan
            self.get_logger().info('Planning...')
            plan_result = self.arm.plan()
            
            if not plan_result:
                self.get_logger().error(f'✗ Planning failed for pose: {pose_name}')
                return False
            
            # Execute
            self.get_logger().info('Executing...')
            self.moveit.execute(plan_result.trajectory, blocking=True)
            
            self.get_logger().info(f'✓ Reached {pose_name}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'✗ Failed: {e}')
            return False
    
    def run_interactive(self):
        """
        Interactive mode - wait for user input and move to positions.
        Similar to old controller interface.
        """
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('INTERACTIVE MODE')
        self.get_logger().info('═══════════════════════════════════════')
        
        while True:
            try:
                # Get input (like old controller)
                user_input = input('\n> Enter position (or "quit"): ').strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    self.get_logger().info('Exiting...')
                    break
                
                # Check if it's a named pose
                named_poses = ['home', 'stow', 'es_ready', 'es_panel_approach', 
                              'es_fine_position', 'sc_ready', 'sc_ground_survey',
                              'sc_sample_collect', 'dm_ready', 'dm_pickup_approach',
                              'dm_pickup_grasp']
                
                if user_input.lower() in named_poses:
                    self.move_to_named_pose(user_input.lower())
                    continue
                
                # Parse position input
                parts = user_input.split()
                
                if len(parts) < 3:
                    self.get_logger().warn('Invalid input! Need at least x y z')
                    continue
                
                # Parse x, y, z (required)
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                
                # Parse rx, ry, rz (optional, default to standard orientation)
                if len(parts) >= 6:
                    rx, ry, rz = float(parts[3]), float(parts[4]), float(parts[5])
                else:
                    rx, ry, rz = 0.0, np.pi, 0.0  # Default orientation
                
                # Move to position (like old IK service)
                success = self.move_to_position(x, y, z, rx, ry, rz)
                
                if success:
                    self.get_logger().info('✓ Ready for next command')
                else:
                    self.get_logger().warn('✗ Try different position')
                
            except ValueError:
                self.get_logger().error('Invalid numbers! Use format: x y z [rx ry rz]')
            except KeyboardInterrupt:
                self.get_logger().info('\nInterrupted - exiting...')
                break
            except Exception as e:
                self.get_logger().error(f'Error: {e}')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = SimpleArmController()
        
        # Check if we should run in interactive mode
        if '--interactive' in sys.argv or len(sys.argv) == 1:
            # Run interactive in a separate thread to allow ROS spinning
            import threading
            
            def spin_node():
                rclpy.spin(controller)
            
            spin_thread = threading.Thread(target=spin_node, daemon=True)
            spin_thread.start()
            
            # Run interactive mode in main thread
            controller.run_interactive()
            
        else:
            # Service mode only
            controller.get_logger().info('Running in service mode...')
            rclpy.spin(controller)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Controller failed: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

