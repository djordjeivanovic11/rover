#!/usr/bin/env python3
"""
Auto-generate MoveIt configuration files from arm parameters.
This script creates all necessary MoveIt config files for planning and control.
"""

import os
import sys
import yaml
import argparse
from pathlib import Path
from textwrap import dedent

def load_arm_params(params_file):
    """Load arm parameters from YAML file."""
    try:
        with open(params_file, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading parameters: {e}")
        sys.exit(1)

def generate_joint_limits_yaml(arm_params, output_dir):
    """Generate joint_limits.yaml file."""
    arm_config = arm_params['arm']
    content = dedent(f"""
    # AUTO-GENERATED JOINT LIMITS
    # Generated from arm_params.yaml - do not edit manually
    
    joint_limits:
    """).strip()
    
    for joint_name in arm_config['joints']:
        pos_limits = arm_config['joint_limits']['position'][joint_name]
        vel_limit = arm_config['joint_limits']['velocity'][joint_name]
        accel_limit = arm_config['joint_limits']['acceleration'][joint_name]
        
        content += f"""
      {joint_name}:
        has_position_limits: true
        min_position: {pos_limits[0]}
        max_position: {pos_limits[1]}
        has_velocity_limits: true
        max_velocity: {vel_limit}
        has_acceleration_limits: true
        max_acceleration: {accel_limit}"""
    
    output_file = output_dir / "joint_limits.yaml"
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Generated: {output_file}")

def generate_kinematics_yaml(arm_params, output_dir):
    """Generate kinematics.yaml file."""
    arm_config = arm_params['arm']
    solver_config = arm_config['moveit']['kinematics_solver']
    
    content = dedent(f"""
    # AUTO-GENERATED KINEMATICS CONFIGURATION
    # Generated from arm_params.yaml - do not edit manually
    
    arm:
      kinematics_solver: {solver_config['solver']}
      kinematics_solver_search_resolution: {solver_config['search_resolution']}
      kinematics_solver_timeout: {solver_config['timeout']}
      kinematics_solver_attempts: {solver_config['attempts']}
      kinematics_solver_max_iterations: {solver_config['max_iterations']}
    """).strip()
    
    output_file = output_dir / "kinematics.yaml"
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Generated: {output_file}")

def generate_ompl_planning_yaml(arm_params, output_dir):
    """Generate ompl_planning.yaml file."""
    arm_config = arm_params['arm']
    planning_config = arm_config['moveit']['planning']
    
    content = dedent(f"""
    # AUTO-GENERATED OMPL PLANNING CONFIGURATION
    # Generated from arm_params.yaml - do not edit manually
    
    planner_configs:
      SBLkConfigDefault:
        type: geometric::SBL
        range: 0.0
      ESTkConfigDefault:
        type: geometric::EST
        range: 0.0
        goal_bias: 0.05
      LBKPIECEkConfigDefault:
        type: geometric::LBKPIECE
        range: 0.0
        border_fraction: 0.9
        min_valid_path_fraction: 0.5
      BKPIECEkConfigDefault:
        type: geometric::BKPIECE
        range: 0.0
        border_fraction: 0.9
        failed_expansion_score_factor: 0.5
        min_valid_path_fraction: 0.2
      KPIECEkConfigDefault:
        type: geometric::KPIECE
        range: 0.0
        goal_bias: 0.05
        border_fraction: 0.9
        failed_expansion_score_factor: 0.5
        min_valid_path_fraction: 0.2
      RRTkConfigDefault:
        type: geometric::RRT
        range: 0.0
        goal_bias: 0.05
      RRTConnectkConfigDefault:
        type: geometric::RRTConnect
        range: 0.0
      RRTstarkConfigDefault:
        type: geometric::RRTstar
        range: 0.0
        goal_bias: 0.05
        delay_collision_checking: 1
      TRRTkConfigDefault:
        type: geometric::TRRT
        range: 0.0
        goal_bias: 0.05
        max_states_failed: 10
        temp_change_factor: 2.0
        min_temperature: 10e-10
        init_temperature: 10e-6
        frountier_threshold: 0.0
        frountierNodeRatio: 0.1
        k_constant: 0.0
      PRMkConfigDefault:
        type: geometric::PRM
        max_nearest_neighbors: 10
      PRMstarkConfigDefault:
        type: geometric::PRMstar
    
    arm:
      default_planner_config: {planning_config['planner']}kConfigDefault
      planner_configs:
        - SBLkConfigDefault
        - ESTkConfigDefault
        - LBKPIECEkConfigDefault
        - BKPIECEkConfigDefault
        - KPIECEkConfigDefault
        - RRTkConfigDefault
        - RRTConnectkConfigDefault
        - RRTstarkConfigDefault
        - TRRTkConfigDefault
        - PRMkConfigDefault
        - PRMstarkConfigDefault
      projection_evaluator: joints(base_rotate,shoulder_pitch)
      longest_valid_segment_fraction: 0.005
    """).strip()
    
    output_file = output_dir / "ompl_planning.yaml"
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Generated: {output_file}")

def generate_controllers_yaml(arm_params, output_dir):
    """Generate controllers.yaml file."""
    arm_config = arm_params['arm']
    controller_config = arm_config['controllers']['joint_trajectory_controller']
    
    joint_list = "', '".join(arm_config['joints'])
    
    content = dedent(f"""
    # AUTO-GENERATED CONTROLLERS CONFIGURATION
    # Generated from arm_params.yaml - do not edit manually
    
    controller_manager:
      ros__parameters:
        update_rate: 100
        
        joint_trajectory_controller:
          type: {controller_config['type']}
        
        joint_state_broadcaster:
          type: joint_state_broadcaster/JointStateBroadcaster
    
    joint_trajectory_controller:
      ros__parameters:
        joints:
          - '{joint_list.replace("', '", "'\n          - '")}'
        
        command_interfaces:
          - {controller_config['command_interfaces'][0]}
        
        state_interfaces:
          - {controller_config['state_interfaces'][0]}
          - {controller_config['state_interfaces'][1]}
        
        state_publish_rate: 50.0
        action_monitor_rate: 20.0
        allow_partial_joints_goal: false
        
        constraints:
          stopped_velocity_tolerance: {controller_config['constraints']['stopped_velocity_tolerance']}
          goal_time: {controller_config['constraints']['goal_time']}""")
    
    # Add individual joint constraints
    for joint_name in arm_config['joints']:
        tolerances = controller_config['constraints'][joint_name]
        content += f"""
          {joint_name}:
            trajectory: {tolerances[0]}
            goal: {tolerances[0]}"""
    
    output_file = output_dir / "controllers.yaml"
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Generated: {output_file}")

def generate_moveit_controllers_yaml(arm_params, output_dir):
    """Generate moveit_controllers.yaml file."""
    arm_config = arm_params['arm']
    
    joint_list = arm_config['joints']
    joint_names = "', '".join(joint_list)
    
    content = dedent(f"""
    # AUTO-GENERATED MOVEIT CONTROLLERS CONFIGURATION
    # Generated from arm_params.yaml - do not edit manually
    
    moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
    
    moveit_simple_controller_manager:
      controller_names:
        - joint_trajectory_controller
      
      joint_trajectory_controller:
        type: FollowJointTrajectory
        action_ns: follow_joint_trajectory
        default: true
        joints:
          - '{joint_names.replace("', '", "'\n          - '")}'
    """).strip()
    
    output_file = output_dir / "moveit_controllers.yaml"
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Generated: {output_file}")

def generate_trajectory_execution_yaml(arm_params, output_dir):
    """Generate trajectory_execution.yaml file."""
    arm_config = arm_params['arm']
    planning_config = arm_config['moveit']['planning']
    
    content = dedent(f"""
    # AUTO-GENERATED TRAJECTORY EXECUTION CONFIGURATION
    # Generated from arm_params.yaml - do not edit manually
    
    trajectory_execution:
      allowed_execution_duration_scaling: 1.2
      allowed_goal_duration_margin: 0.5
      allowed_start_tolerance: 0.01
      
    planning_scene_monitor:
      publish_planning_scene: true
      publish_geometry_updates: true
      publish_state_updates: true
      publish_transforms_updates: true
      planning_scene_publish_rate: 10.0
      
    move_group:
      planning_attempts: {planning_config['planning_attempts']}
      planning_time: {planning_config['planning_time']}
      max_velocity_scaling_factor: {planning_config['max_velocity_scaling_factor']}
      max_acceleration_scaling_factor: {planning_config['max_acceleration_scaling_factor']}
      jiggle_fraction: 0.05
      max_safe_path_cost: 1
      publish_monitored_planning_scene: true
    """).strip()
    
    output_file = output_dir / "trajectory_execution.yaml"
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Generated: {output_file}")

def generate_planning_scene_monitor_yaml(arm_params, output_dir):
    """Generate planning_scene_monitor.yaml file."""
    content = dedent("""
    # AUTO-GENERATED PLANNING SCENE MONITOR CONFIGURATION
    # Generated from arm_params.yaml - do not edit manually
    
    planning_scene_monitor:
      publish_planning_scene: true
      publish_geometry_updates: true
      publish_state_updates: true
      publish_transforms_updates: true
      planning_scene_publish_rate: 10.0
      
    octomap_monitor:
      octomap_frame: world
      octomap_resolution: 0.025
      max_range: 5.0
    """).strip()
    
    output_file = output_dir / "planning_scene_monitor.yaml"
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Generated: {output_file}")

def generate_sensors_3d_yaml(arm_params, output_dir):
    """Generate sensors_3d.yaml file."""
    content = dedent("""
    # AUTO-GENERATED 3D SENSORS CONFIGURATION
    # Generated from arm_params.yaml - do not edit manually
    
    sensors:
      - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
        point_cloud_topic: /camera/depth/color/points
        max_range: 5.0
        point_subsample: 1
        padding_offset: 0.1
        padding_scale: 1.0
        max_update_rate: 1.0
        filtered_cloud_topic: filtered_cloud
    """).strip()
    
    output_file = output_dir / "sensors_3d.yaml"
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"Generated: {output_file}")

def generate_all_configs(params_file, output_dir):
    """Generate all MoveIt configuration files."""
    print(f"Loading parameters from: {params_file}")
    arm_params = load_arm_params(params_file)
    
    print(f"Generating MoveIt configuration files in: {output_dir}")
    
    # Ensure output directory exists
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Generate all configuration files
    generate_joint_limits_yaml(arm_params, output_dir)
    generate_kinematics_yaml(arm_params, output_dir)
    generate_ompl_planning_yaml(arm_params, output_dir)
    generate_controllers_yaml(arm_params, output_dir)
    generate_moveit_controllers_yaml(arm_params, output_dir)
    generate_trajectory_execution_yaml(arm_params, output_dir)
    generate_planning_scene_monitor_yaml(arm_params, output_dir)
    generate_sensors_3d_yaml(arm_params, output_dir)
    
    print("MoveIt configuration generation complete!")

def main():
    parser = argparse.ArgumentParser(description='Generate MoveIt configuration from arm parameters')
    parser.add_argument('--params', 
                       default='config/arm_params.yaml',
                       help='Path to arm parameters YAML file')
    parser.add_argument('--output-dir',
                       default='../moveit_config_template/config',
                       help='Output directory for MoveIt config files')
    
    args = parser.parse_args()
    
    # Convert to absolute paths
    script_dir = Path(__file__).parent
    params_file = script_dir / args.params
    output_dir = script_dir / args.output_dir
    
    generate_all_configs(str(params_file), output_dir)

if __name__ == '__main__':
    main() 