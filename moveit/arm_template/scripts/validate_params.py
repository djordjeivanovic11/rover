#!/usr/bin/env python3
"""
Validate arm parameters for consistency and completeness.
This script checks arm_params.yaml for common issues and missing values.
"""

import os
import sys
import yaml
import argparse
from pathlib import Path
import math

def load_arm_params(params_file):
    """Load arm parameters from YAML file."""
    try:
        with open(params_file, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"❌ Error loading parameters: {e}")
        return None

def validate_required_params(arm_params):
    """Validate that all required parameters are present."""
    print("🔍 Validating required parameters...")
    
    required_paths = [
        'arm.name',
        'arm.joints',
        'arm.joint_limits.position',
        'arm.joint_limits.velocity',
        'arm.joint_limits.effort',
        'arm.kinematics',
        'arm.link_masses',
        'arm.link_geometries',
        'arm.moveit.planning_groups'
    ]
    
    errors = []
    
    for path in required_paths:
        keys = path.split('.')
        current = arm_params
        
        try:
            for key in keys:
                current = current[key]
            print(f"  ✓ {path}")
        except (KeyError, TypeError):
            errors.append(f"  ❌ Missing required parameter: {path}")
    
    if errors:
        print("Required parameter errors:")
        for error in errors:
            print(error)
        return False
    
    print("✅ All required parameters present")
    return True

def validate_joint_consistency(arm_params):
    """Validate joint-related parameters for consistency."""
    print("🔍 Validating joint consistency...")
    
    arm_config = arm_params['arm']
    joint_names = arm_config['joints']
    errors = []
    
    # Check joint limits consistency
    for limit_type in ['position', 'velocity', 'effort']:
        if limit_type in arm_config['joint_limits']:
            limits = arm_config['joint_limits'][limit_type]
            
            for joint in joint_names:
                if joint not in limits:
                    errors.append(f"❌ Missing {limit_type} limit for joint: {joint}")
                else:
                    if limit_type == 'position':
                        pos_limits = limits[joint]
                        if len(pos_limits) != 2:
                            errors.append(f"❌ Position limits for {joint} must have [min, max]")
                        elif pos_limits[0] >= pos_limits[1]:
                            errors.append(f"❌ Invalid position limits for {joint}: min >= max")
                    else:
                        if limits[joint] <= 0:
                            errors.append(f"❌ {limit_type} limit for {joint} must be positive")
    
    # Check joint axes
    if 'joint_axes' in arm_config['kinematics']:
        axes = arm_config['kinematics']['joint_axes']
        for joint in joint_names:
            if joint not in axes:
                errors.append(f"❌ Missing joint axis for: {joint}")
            else:
                axis = axes[joint]
                if len(axis) != 3:
                    errors.append(f"❌ Joint axis for {joint} must have 3 components")
                elif all(abs(x) < 1e-6 for x in axis):
                    errors.append(f"❌ Joint axis for {joint} cannot be zero vector")
    
    if errors:
        print("Joint consistency errors:")
        for error in errors:
            print(f"  {error}")
        return False
    
    print("✅ Joint parameters are consistent")
    return True

def validate_physical_parameters(arm_params):
    """Validate physical parameters (masses, inertias, etc.)."""
    print("🔍 Validating physical parameters...")
    
    arm_config = arm_params['arm']
    errors = []
    
    # Validate masses
    if 'link_masses' in arm_config:
        masses = arm_config['link_masses']
        for link, mass in masses.items():
            if mass <= 0:
                errors.append(f"❌ Mass for {link} must be positive, got: {mass}")
    
    # Validate inertias
    if 'link_inertias' in arm_config:
        inertias = arm_config['link_inertias']
        for link, inertia in inertias.items():
            if len(inertia) != 6:
                errors.append(f"❌ Inertia for {link} must have 6 components [ixx, iyy, izz, ixy, ixz, iyz]")
            else:
                # Check that principal moments are positive
                ixx, iyy, izz = inertia[0], inertia[1], inertia[2]
                if ixx <= 0 or iyy <= 0 or izz <= 0:
                    errors.append(f"❌ Principal moments of inertia for {link} must be positive")
    
    # Validate geometries
    if 'link_geometries' in arm_config:
        geometries = arm_config['link_geometries']
        for link, geometry in geometries.items():
            if len(geometry) != 3:
                errors.append(f"❌ Geometry for {link} must have 3 dimensions [x, y, z]")
            else:
                if any(dim <= 0 for dim in geometry):
                    errors.append(f"❌ All geometry dimensions for {link} must be positive")
    
    if errors:
        print("Physical parameter errors:")
        for error in errors:
            print(f"  {error}")
        return False
    
    print("✅ Physical parameters are valid")
    return True

def validate_kinematic_parameters(arm_params):
    """Validate kinematic chain parameters."""
    print("🔍 Validating kinematic parameters...")
    
    arm_config = arm_params['arm']
    errors = []
    
    if 'kinematics' in arm_config:
        kinematics = arm_config['kinematics']
        
        # Check link lengths are positive
        length_params = [
            'base_height', 'shoulder_offset', 'upper_arm_length',
            'forearm_length', 'wrist_offset', 'tool_flange_length'
        ]
        
        for param in length_params:
            if param in kinematics:
                if kinematics[param] <= 0:
                    errors.append(f"❌ {param} must be positive, got: {kinematics[param]}")
    
    # Validate workspace constraints
    if 'workspace' in arm_config:
        workspace = arm_config['workspace']
        
        if 'max_reach' in workspace and 'min_reach' in workspace:
            if workspace['max_reach'] <= workspace['min_reach']:
                errors.append("❌ max_reach must be greater than min_reach")
        
        if 'max_height' in workspace and 'min_height' in workspace:
            if workspace['max_height'] <= workspace['min_height']:
                errors.append("❌ max_height must be greater than min_height")
    
    if errors:
        print("Kinematic parameter errors:")
        for error in errors:
            print(f"  {error}")
        return False
    
    print("✅ Kinematic parameters are valid")
    return True

def validate_mission_poses(arm_params):
    """Validate mission-specific poses."""
    print("🔍 Validating mission poses...")
    
    arm_config = arm_params['arm']
    errors = []
    
    if 'mission_poses' in arm_config:
        poses = arm_config['mission_poses']
        joint_names = arm_config['joints']
        joint_limits = arm_config['joint_limits']['position']
        
        for pose_name, joint_values in poses.items():
            if len(joint_values) != len(joint_names):
                errors.append(f"❌ Pose '{pose_name}' has {len(joint_values)} values, expected {len(joint_names)}")
                continue
            
            # Check joint limits
            for i, (joint, value) in enumerate(zip(joint_names, joint_values)):
                if joint in joint_limits:
                    min_pos, max_pos = joint_limits[joint]
                    if value < min_pos or value > max_pos:
                        errors.append(f"❌ Pose '{pose_name}': joint '{joint}' value {value} outside limits [{min_pos}, {max_pos}]")
    
    if errors:
        print("Mission pose errors:")
        for error in errors:
            print(f"  {error}")
        return False
    
    print("✅ Mission poses are valid")
    return True

def validate_moveit_config(arm_params):
    """Validate MoveIt-specific configuration."""
    print("🔍 Validating MoveIt configuration...")
    
    arm_config = arm_params['arm']
    errors = []
    
    if 'moveit' in arm_config:
        moveit_config = arm_config['moveit']
        
        # Validate planning groups
        if 'planning_groups' in moveit_config:
            for group in moveit_config['planning_groups']:
                if 'joints' not in group:
                    errors.append(f"❌ Planning group missing 'joints' field")
                    continue
                
                # Check that all joints exist
                group_joints = group['joints']
                arm_joints = arm_config['joints']
                
                for joint in group_joints:
                    if joint not in arm_joints:
                        errors.append(f"❌ Planning group references unknown joint: {joint}")
        
        # Validate kinematics solver config
        if 'kinematics_solver' in moveit_config:
            solver = moveit_config['kinematics_solver']
            
            required_solver_params = ['solver', 'search_resolution', 'timeout']
            for param in required_solver_params:
                if param not in solver:
                    errors.append(f"❌ Missing kinematics solver parameter: {param}")
                elif param in ['search_resolution', 'timeout'] and solver[param] <= 0:
                    errors.append(f"❌ Kinematics solver {param} must be positive")
    
    if errors:
        print("MoveIt configuration errors:")
        for error in errors:
            print(f"  {error}")
        return False
    
    print("✅ MoveIt configuration is valid")
    return True

def validate_safety_parameters(arm_params):
    """Validate safety-related parameters."""
    print("🔍 Validating safety parameters...")
    
    arm_config = arm_params['arm']
    errors = []
    
    if 'safety' in arm_config:
        safety = arm_config['safety']
        
        # Check velocity/acceleration limits
        velocity_params = [
            'max_linear_velocity', 'max_angular_velocity',
            'max_linear_acceleration', 'max_angular_acceleration'
        ]
        
        for param in velocity_params:
            if param in safety:
                if safety[param] <= 0:
                    errors.append(f"❌ Safety parameter {param} must be positive")
        
        # Check force/torque limits
        force_params = ['max_force', 'max_torque']
        for param in force_params:
            if param in safety:
                if safety[param] <= 0:
                    errors.append(f"❌ Safety parameter {param} must be positive")
        
        # Check collision margin
        if 'collision_margin' in safety:
            if safety['collision_margin'] <= 0:
                errors.append("❌ Collision margin must be positive")
    
    if errors:
        print("Safety parameter errors:")
        for error in errors:
            print(f"  {error}")
        return False
    
    print("✅ Safety parameters are valid")
    return True

def generate_summary_report(arm_params):
    """Generate a summary report of the arm configuration."""
    print("\n📋 Configuration Summary Report:")
    print("=" * 50)
    
    arm_config = arm_params['arm']
    
    print(f"Robot Name: {arm_config['name']}")
    print(f"DOF: {arm_config['dof']}")
    print(f"Number of Joints: {len(arm_config['joints'])}")
    
    if 'kinematics' in arm_config:
        kinematics = arm_config['kinematics']
        total_reach = sum([
            kinematics.get('upper_arm_length', 0),
            kinematics.get('forearm_length', 0),
            kinematics.get('wrist_offset', 0),
            kinematics.get('tool_flange_length', 0)
        ])
        print(f"Approximate Max Reach: {total_reach:.3f} m")
    
    if 'link_masses' in arm_config:
        total_mass = sum(arm_config['link_masses'].values())
        print(f"Total Mass: {total_mass:.2f} kg")
    
    if 'mission_poses' in arm_config:
        print(f"Predefined Poses: {len(arm_config['mission_poses'])}")
    
    if 'gripper' in arm_config and arm_config['gripper']:
        gripper = arm_config['gripper']
        print(f"Gripper: {gripper['name']} (max opening: {gripper['max_opening']} m)")
    
    print("=" * 50)

def validate_arm_params(params_file):
    """Run complete validation of arm parameters."""
    print(f"🔧 Validating arm parameters: {params_file}")
    print("=" * 60)
    
    # Load parameters
    arm_params = load_arm_params(params_file)
    if not arm_params:
        return False
    
    # Run all validations
    validations = [
        validate_required_params,
        validate_joint_consistency,
        validate_physical_parameters,
        validate_kinematic_parameters,
        validate_mission_poses,
        validate_moveit_config,
        validate_safety_parameters
    ]
    
    all_valid = True
    for validation_func in validations:
        try:
            if not validation_func(arm_params):
                all_valid = False
        except Exception as e:
            print(f"❌ Validation error in {validation_func.__name__}: {e}")
            all_valid = False
        print()  # Add spacing between validation sections
    
    # Generate summary
    generate_summary_report(arm_params)
    
    # Final result
    if all_valid:
        print("🎉 All validations passed! Parameters are ready for use.")
        return True
    else:
        print("💥 Validation failed! Please fix the errors above.")
        return False

def main():
    parser = argparse.ArgumentParser(description='Validate arm parameters')
    parser.add_argument('--params', 
                       default='config/arm_params.yaml',
                       help='Path to arm parameters YAML file')
    parser.add_argument('--quiet', action='store_true',
                       help='Only show errors and final result')
    
    args = parser.parse_args()
    
    # Convert to absolute path
    script_dir = Path(__file__).parent
    params_file = script_dir / args.params
    
    if not params_file.exists():
        print(f"❌ Parameters file not found: {params_file}")
        sys.exit(1)
    
    # Run validation
    success = validate_arm_params(str(params_file))
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main() 