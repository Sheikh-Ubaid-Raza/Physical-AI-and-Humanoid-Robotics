---
sidebar_position: 1
title: "Gazebo Environment Setup: URDF/SDF Formats, Physics Simulation"
---

# Gazebo Environment Setup: URDF/SDF Formats, Physics Simulation

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up Gazebo simulation environments for robot testing
- Convert URDF models to SDF format for Gazebo
- Configure physics parameters for realistic simulation
- Implement robot controllers that work in simulation
- Design simulation scenarios that match real-world conditions

## Prerequisites

- Understanding of URDF robot description format (covered in Week 5)
- Basic knowledge of robot physics and dynamics
- Experience with ROS 2 launch files (covered in Week 5)

## Theory

Gazebo is a physics-based simulator that enables testing of robot software in realistic environments. It provides high-fidelity physics simulation, high-quality graphics rendering, and sensor simulation that closely matches real-world behavior.

### URDF vs SDF

While URDF (Unified Robot Description Format) is the standard for robot description in ROS, Gazebo uses SDF (Simulation Description Format). Although URDF can be loaded directly into Gazebo, SDF offers more features specifically for simulation including:

- Physics parameters (friction, damping, restitution)
- Sensor definitions with noise models
- Plugin specifications for Gazebo-specific functionality
- Environment and world descriptions

### Physics Simulation

Gazebo uses the ODE (Open Dynamics Engine) physics engine by default, though it can be configured to use other engines like Bullet or DART. Physics parameters that affect simulation realism include:

- Gravity: Standard value is 9.81 m/s² downward
- Friction: Determines how surfaces interact (mu1, mu2 parameters)
- Damping: Reduces motion over time (angular and linear damping)
- Restitution: Determines bounciness of collisions (coefficient of restitution)

### Gazebo Plugins

Gazebo plugins are shared libraries that provide additional functionality to models and sensors in the simulation. Common plugins include:
- Joint state publishers
- Robot state publishers
- Controller interfaces (for ROS 2 integration)
- Sensor drivers (camera, LIDAR, IMU, etc.)

## Code Example 1: SDF Conversion and World Definition

```xml
<!-- worlds/my_robot_world.sdf -->
<!-- Purpose: Define a simulation world with robot and environment -->
<!-- Setup Instructions: Place in worlds directory of Gazebo package -->
<!-- Run: gazebo worlds/my_robot_world.sdf -->

<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_robot_world">

    <!-- Physics Engine Configuration -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.3 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.7 0.7 0.7 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include Robot Model -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>

    <!-- Obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="box_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.5</mu>
                <mu2>0.5</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
            <specular>0.8 0.2 0.2 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.083</iyy>
            <iyz>0</iyz>
            <izz>0.042</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

**Expected Output:**
```
# When running Gazebo with this world file:
Gazebo multi-robot simulator, version 11.0.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2.0 License.
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Loading world file [/path/to/worlds/my_robot_world.sdf]
[Msg] Successfully launched Gazebo with world: my_robot_world.sdf
```

## Code Example 2: Robot Simulation Launch File

```python
# launch/robot_gazebo.launch.py
# Purpose: Launch robot in Gazebo with controllers
# Setup Instructions: Place in launch directory of ROS 2 package
# Run: ros2 launch my_robot_package robot_gazebo.launch.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_robot = get_package_share_directory('my_robot_package')

    # World file
    world_file = os.path.join(
        get_package_share_directory('my_robot_package'),
        'worlds',
        'my_robot_world.sdf'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Diff drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])
```

**Expected Output:**
```
[INFO] [launch]: All processes have started successfully
[gazebo-1] Gazebo is launched with the custom world
[INFO] [spawn_entity.py]: Spawned entity [my_robot]
[robot_state_publisher-2] Publishing robot state
[INFO] [controller_manager]: Loaded joint_state_broadcaster
[INFO] [controller_manager]: Loaded diff_drive_controller
```

## Hands-on Exercises

1. Create a Gazebo world with multiple obstacles and simulate robot navigation through the environment
2. Configure physics parameters to simulate different ground conditions (ice, sand, grass) and observe the effect on robot mobility
3. Implement a sensor-equipped robot model in Gazebo and verify that sensor data matches expected real-world behavior

## Summary

Gazebo provides a powerful simulation environment for testing robot software before deployment to physical hardware. Proper configuration of physics parameters, world environments, and robot models is crucial for achieving realistic simulation that translates well to real-world performance. The integration of Gazebo with ROS 2 enables seamless development and testing of robot applications.

## Hardware Requirements

This chapter can be completed using simulation environments. For physical implementation, the following hardware is recommended:
- Computer with Gazebo Garden or Fortress installed
- Adequate GPU for rendering (OpenGL 2.1+)
- Robot hardware compatible with URDF/SDF models for validation
- Simulation-to-reality comparison setup (optional for basic exercises)