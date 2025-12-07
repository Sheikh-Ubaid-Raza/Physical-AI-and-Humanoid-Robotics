---
sidebar_position: 1
title: "Launch Files, Parameter Management, URDF Basics"
---

# Launch Files, Parameter Management, URDF Basics

## Learning Objectives

By the end of this chapter, you will be able to:
- Create and configure ROS 2 launch files for complex robot systems
- Manage parameters effectively in ROS 2 systems
- Define robot models using URDF (Unified Robot Description Format)
- Load and visualize robot models in simulation environments
- Integrate launch files with URDF models and parameter configurations

## Prerequisites

- Understanding of ROS 2 architecture and nodes (covered in Week 3)
- Experience with building ROS 2 packages (covered in Week 4)
- Basic knowledge of XML format

## Theory

### Launch Files

Launch files in ROS 2 provide a way to start multiple nodes with specific configurations simultaneously. They allow for complex robot systems to be launched with a single command, managing node parameters, remappings, and lifecycle management.

Launch files can be written in Python or XML. Python launch files offer more flexibility and programmability, while XML launch files provide a declarative approach that is easier to read and maintain.

### Parameter Management

Parameters in ROS 2 provide a way to configure node behavior without recompilation. Parameters can be set at launch time, loaded from YAML files, or dynamically changed during runtime. Proper parameter management is crucial for robot systems that need to operate in different environments or configurations.

### URDF (Unified Robot Description Format)

URDF is an XML-based format used to describe robot models in ROS. It defines the robot's physical properties including:
- Kinematic structure (joints and links)
- Visual and collision geometry
- Inertial properties
- Materials and colors

URDF files are essential for robot simulation, visualization, and planning algorithms.

## Code Example 1: Python Launch File with Parameters

```python
# launch/robot_system.launch.py
# Purpose: Launch a complete robot system with multiple nodes and parameters
# Setup Instructions: Place in launch directory of ROS 2 package
# Run: ros2 launch my_robot_package robot_system.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot controller node
    robot_controller = Node(
        package='my_robot_package',
        executable='robot_controller',
        name='robot_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'control_rate': 50.0},  # Hz
            {'max_linear_velocity': 1.0},  # m/s
            {'max_angular_velocity': 1.0},  # rad/s
        ],
        output='screen'
    )

    # Sensor processor node
    sensor_processor = Node(
        package='my_robot_package',
        executable='sensor_processor',
        name='sensor_processor',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'laser_topic': '/scan'},
            {'camera_topic': '/camera/color/image_raw'},
        ],
        output='screen'
    )

    # Launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(robot_controller)
    ld.add_action(sensor_processor)

    return ld
```

**Expected Output:**
```
[INFO] [launch]: All processes have started successfully
[robot_controller-1] [INFO] [1620123456.789] [robot_controller]: Robot controller initialized with parameters
[robot_controller-1] [INFO] [1620123456.789] [robot_controller]: Control rate: 50.0 Hz
[sensor_processor-2] [INFO] [1620123456.790] [sensor_processor]: Sensor processor started
[INFO] [launch]: Process robot_controller-1 terminated successfully
[INFO] [launch]: Process sensor_processor-2 terminated successfully
```

## Code Example 2: URDF Robot Model Definition

```xml
<!-- urdf/my_robot.urdf -->
<!-- Purpose: Define a simple differential drive robot model -->
<!-- Setup Instructions: Place in urdf directory of ROS 2 package -->
<!-- Run: ros2 run robot_state_publisher robot_state_publisher my_robot.urdf -->

<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.15 -0.2 0" rpy="1.570796 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.15 0.2 0" rpy="1.570796 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Laser Sensor -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="laser_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

</robot>
```

**Expected Output:**
```
# When loaded with robot_state_publisher:
[INFO] [robot_state_publisher]: Robot initialized
[INFO] [robot_state_publisher]: Publishing transforms for robot model
# Robot model can be visualized in RViz2 or Gazebo
```

## Hands-on Exercises

1. Create a launch file that starts a robot simulation with multiple sensors and loads robot parameters from a YAML file
2. Define a URDF model for a simple robotic arm with at least 3 joints and visualize it in RViz
3. Implement a parameter server node that manages robot configuration parameters and provides a service interface for parameter updates

## Summary

Launch files, parameter management, and URDF are fundamental components of any ROS 2 robot system. They provide the infrastructure needed to configure, launch, and manage complex robot applications. Proper use of these tools enables flexible and maintainable robot software architectures.

## Hardware Requirements

This chapter can be completed using simulation environments. For physical implementation, the following hardware is recommended:
- Computer with ROS 2 installed (Ubuntu 22.04 with ROS 2 Humble Hawksbill)
- RViz2 for visualization
- Robot simulation environment (e.g., Gazebo Garden)
- Robot hardware with appropriate URDF model (optional for basic exercises)