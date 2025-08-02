#!/usr/bin/env python3
"""
Auto-generate SRDF file from arm parameters.
This script reads arm_params.yaml and creates a properly configured SRDF
for MoveIt motion planning.
"""

import os
import sys
import yaml
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom

def load_arm_params(params_file):
    """Load arm parameters from YAML file."""
    try:
        with open(params_file, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading parameters: {e}")
        sys.exit(1)

def create_srdf_root(arm_params):
    """Create root SRDF element with robot name."""
    robot = ET.Element('robot')
    robot.set('name', arm_params['arm']['name'])
    return robot

def add_planning_groups(root, arm_params):
    """Add planning groups to SRDF."""
    arm_config = arm_params['arm']
    
    for group_config in arm_config['moveit']['planning_groups']:
        group = ET.SubElement(root, 'group')
        group.set('name', group_config['name'])
        
        # Add joints to group
        for joint_name in group_config['joints']:
            joint_elem = ET.SubElement(group, 'joint')
            joint_elem.set('name', joint_name)
        
        # Add end effector if specified
        if 'end_effector' in group_config:
            # Create end effector group
            ee_group = ET.SubElement(root, 'group')
            ee_group.set('name', f"{group_config['name']}_end_effector")
            
            # Add tool flange link
            link_elem = ET.SubElement(ee_group, 'link')
            link_elem.set('name', group_config['end_effector'])
            
            # Add gripper if present
            if 'gripper' in arm_config and arm_config['gripper'] is not None:
                gripper_name = arm_config['gripper']['name']
                
                # Add gripper links
                gripper_links = [
                    f"{gripper_name}_palm",
                    f"{gripper_name}_left_finger",
                    f"{gripper_name}_right_finger",
                    f"{gripper_name}_left_fingertip",
                    f"{gripper_name}_right_fingertip"
                ]
                
                for link_name in gripper_links:
                    link_elem = ET.SubElement(ee_group, 'link')
                    link_elem.set('name', link_name)
            
            # Associate end effector with main group
            ee_elem = ET.SubElement(group, 'group')
            ee_elem.set('name', f"{group_config['name']}_end_effector")

def add_group_states(root, arm_params):
    """Add predefined group states (named poses)."""
    arm_config = arm_params['arm']
    
    for pose_name, joint_values in arm_config['mission_poses'].items():
        group_state = ET.SubElement(root, 'group_state')
        group_state.set('name', pose_name)
        group_state.set('group', 'arm')  # Assuming main group is 'arm'
        
        # Add joint values
        joint_names = arm_config['joints']
        for i, joint_name in enumerate(joint_names):
            if i < len(joint_values):
                joint_elem = ET.SubElement(group_state, 'joint')
                joint_elem.set('name', joint_name)
                joint_elem.set('value', str(joint_values[i]))

def add_virtual_joint(root, arm_params):
    """Add virtual joint to connect robot to world."""
    virtual_joint = ET.SubElement(root, 'virtual_joint')
    virtual_joint.set('name', 'virtual_joint')
    virtual_joint.set('type', 'fixed')
    virtual_joint.set('parent_frame', 'world')
    virtual_joint.set('child_link', arm_params['arm']['base_frame'])

def add_disable_collisions(root, arm_params):
    """Add collision matrix to disable unnecessary collision checks."""
    arm_config = arm_params['arm']
    links = arm_config['links']
    
    # Disable collisions between adjacent links
    adjacent_pairs = [
        (links[0], links[1]),  # base_link - shoulder_link
        (links[1], links[2]),  # shoulder_link - upper_arm_link
        (links[2], links[3]),  # upper_arm_link - forearm_link
        (links[3], links[4]),  # forearm_link - wrist_link
        (links[4], links[5]),  # wrist_link - tool_flange
    ]
    
    for link1, link2 in adjacent_pairs:
        disable_elem = ET.SubElement(root, 'disable_collisions')
        disable_elem.set('link1', link1)
        disable_elem.set('link2', link2)
        disable_elem.set('reason', 'Adjacent')
    
    # Disable collisions for gripper internal links
    if 'gripper' in arm_config and arm_config['gripper'] is not None:
        gripper_name = arm_config['gripper']['name']
        
        gripper_pairs = [
            (f"{gripper_name}_palm", f"{gripper_name}_left_finger"),
            (f"{gripper_name}_palm", f"{gripper_name}_right_finger"),
            (f"{gripper_name}_left_finger", f"{gripper_name}_left_fingertip"),
            (f"{gripper_name}_right_finger", f"{gripper_name}_right_fingertip"),
        ]
        
        for link1, link2 in gripper_pairs:
            disable_elem = ET.SubElement(root, 'disable_collisions')
            disable_elem.set('link1', link1)
            disable_elem.set('link2', link2)
            disable_elem.set('reason', 'Adjacent')

def add_passive_joints(root, arm_params):
    """Add passive joints (if any) that MoveIt should not control."""
    # Example: if gripper has mimic joints
    arm_config = arm_params['arm']
    
    if 'gripper' in arm_config and arm_config['gripper'] is not None:
        gripper_name = arm_config['gripper']['name']
        
        # Right finger joint is typically passive (mimics left)
        passive_joint = ET.SubElement(root, 'passive_joint')
        passive_joint.set('name', f"{gripper_name}_right_finger_joint")

def prettify_xml(elem):
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(elem, 'unicode')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def generate_srdf(params_file, output_file):
    """Generate SRDF file from parameters."""
    print(f"Loading parameters from: {params_file}")
    arm_params = load_arm_params(params_file)
    
    print("Generating SRDF...")
    
    # Create SRDF root
    root = create_srdf_root(arm_params)
    
    # Add SRDF components
    add_virtual_joint(root, arm_params)
    add_planning_groups(root, arm_params)
    add_group_states(root, arm_params)
    add_disable_collisions(root, arm_params)
    add_passive_joints(root, arm_params)
    
    # Write to file
    print(f"Writing SRDF to: {output_file}")
    
    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    # Write prettified XML
    with open(output_file, 'w') as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<!--\n')
        f.write('  AUTO-GENERATED SRDF FILE\n')
        f.write('  Generated from arm_params.yaml\n')
        f.write('  Do not edit manually - regenerate using generate_srdf.py\n')
        f.write('-->\n')
        
        # Write the prettified XML (skip the first line which includes XML declaration)
        pretty_xml = prettify_xml(root)
        lines = pretty_xml.split('\n')[1:]  # Skip first line
        f.write('\n'.join(lines))
    
    print("SRDF generation complete!")
    
    # Validate the generated SRDF
    validate_srdf(output_file, arm_params)

def validate_srdf(srdf_file, arm_params):
    """Basic validation of generated SRDF."""
    print("Validating generated SRDF...")
    
    try:
        tree = ET.parse(srdf_file)
        root = tree.getroot()
        
        # Check that we have essential elements
        groups = root.findall('group')
        group_states = root.findall('group_state')
        virtual_joints = root.findall('virtual_joint')
        
        print(f"  Found {len(groups)} planning groups")
        print(f"  Found {len(group_states)} group states")
        print(f"  Found {len(virtual_joints)} virtual joints")
        
        # Check that main planning group exists
        main_groups = [g for g in groups if g.get('name') == 'arm']
        if not main_groups:
            print("  WARNING: No 'arm' planning group found")
        else:
            print("  ✓ Main 'arm' planning group found")
        
        # Check joint count in main group
        if main_groups:
            joints_in_group = len(main_groups[0].findall('joint'))
            expected_joints = len(arm_params['arm']['joints'])
            if joints_in_group == expected_joints:
                print(f"  ✓ Correct number of joints in planning group ({joints_in_group})")
            else:
                print(f"  WARNING: Expected {expected_joints} joints, found {joints_in_group}")
        
        print("SRDF validation complete!")
        
    except Exception as e:
        print(f"SRDF validation failed: {e}")

def main():
    parser = argparse.ArgumentParser(description='Generate SRDF from arm parameters')
    parser.add_argument('--params', 
                       default='config/arm_params.yaml',
                       help='Path to arm parameters YAML file')
    parser.add_argument('--output',
                       default='../moveit_config_template/srdf/arm_template.srdf',
                       help='Output SRDF file path')
    parser.add_argument('--validate-only', action='store_true',
                       help='Only validate existing SRDF file')
    
    args = parser.parse_args()
    
    # Convert to absolute paths
    script_dir = Path(__file__).parent
    params_file = script_dir / args.params
    output_file = script_dir / args.output
    
    if args.validate_only:
        arm_params = load_arm_params(str(params_file))
        validate_srdf(str(output_file), arm_params)
    else:
        generate_srdf(str(params_file), str(output_file))

if __name__ == '__main__':
    main() 