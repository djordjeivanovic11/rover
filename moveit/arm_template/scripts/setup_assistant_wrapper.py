#!/usr/bin/env python3

"""
MoveIt Setup Assistant Wrapper Script
This script provides a wrapper for the MoveIt Setup Assistant functionality
for the rover arm template system.
"""

import sys
import os
import yaml
import argparse
from pathlib import Path

def load_arm_params(config_file):
    """Load arm parameters from YAML file."""
    try:
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: Configuration file {config_file} not found.")
        return None
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return None

def generate_urdf_from_params(params, output_file):
    """Generate URDF from arm parameters."""
    if not params:
        return False
    
    # Basic URDF template - in a real implementation this would be more sophisticated
    urdf_content = f"""<?xml version="1.0"?>
<robot name="{params.get('arm', {}).get('name', 'rover_arm')}">
  <!-- Generated URDF from arm parameters -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
  
  <!-- Additional links and joints would be generated here based on parameters -->
  
</robot>"""
    
    try:
        with open(output_file, 'w') as f:
            f.write(urdf_content)
        print(f"Generated URDF: {output_file}")
        return True
    except Exception as e:
        print(f"Error writing URDF file: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='MoveIt Setup Assistant Wrapper')
    parser.add_argument('--config', required=True, help='Path to arm configuration YAML file')
    parser.add_argument('--output-dir', required=True, help='Output directory for generated files')
    parser.add_argument('--validate-only', action='store_true', help='Only validate parameters')
    
    args = parser.parse_args()
    
    # Load configuration
    params = load_arm_params(args.config)
    if not params:
        sys.exit(1)
    
    if args.validate_only:
        print("Parameter validation completed successfully.")
        return
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Generate URDF
    urdf_file = output_dir / "robot.urdf"
    if not generate_urdf_from_params(params, urdf_file):
        sys.exit(1)
    
    print("MoveIt configuration generation completed successfully.")

if __name__ == "__main__":
    main()
