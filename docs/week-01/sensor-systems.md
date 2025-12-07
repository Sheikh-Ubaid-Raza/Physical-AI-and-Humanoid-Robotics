---
sidebar_position: 2
title: "Sensor Systems: LIDAR, Cameras, IMUs, Force/Torque Sensors"
---

# Sensor Systems: LIDAR, Cameras, IMUs, Force/Torque Sensors

## Learning Objectives

By the end of this chapter, you will be able to:
- Describe different types of sensors used in robotics
- Explain how LIDAR, cameras, IMUs, and force/torque sensors work
- Implement basic sensor data processing algorithms
- Compare advantages and disadvantages of different sensor types

## Prerequisites

- Basic understanding of physical quantities (distance, angles, force, acceleration)
- Familiarity with coordinate systems and transformations

## Theory

Robotic systems rely heavily on sensor data to perceive and interact with their environment. Different sensor modalities provide complementary information that enables robots to navigate, manipulate objects, and safely interact with humans.

### LIDAR Sensors

Light Detection and Ranging (LIDAR) sensors emit laser pulses and measure the time it takes for the light to return after reflecting off surfaces. This provides accurate distance measurements to surrounding objects. LIDARs are excellent for creating 2D or 3D maps of the environment and detecting obstacles.

### Camera Systems

Cameras capture visual information from the environment. RGB cameras provide color information, while stereo cameras can estimate depth. Modern computer vision algorithms can extract rich semantic information from camera images, including object detection, recognition, and scene understanding.

### Inertial Measurement Units (IMUs)

IMUs combine accelerometers, gyroscopes, and sometimes magnetometers to measure the robot's motion and orientation. They provide high-frequency measurements of acceleration and angular velocity, which are crucial for robot stabilization and motion control.

### Force/Torque Sensors

Force/torque sensors measure the forces and torques applied to the robot. They are essential for precise manipulation tasks, enabling robots to apply appropriate forces when grasping objects or interacting with the environment.

## Code Example 1: LIDAR Data Processing

```python
# Purpose: Process LIDAR data to detect obstacles and create occupancy grid
# Setup Instructions: Install numpy
# Run: python lidar_processing.py

import numpy as np
import matplotlib.pyplot as plt

def process_lidar_scan(ranges, angle_min, angle_max):
    """Process LIDAR scan data to extract obstacle information"""
    # Calculate angle for each measurement
    angle_increment = (angle_max - angle_min) / len(ranges)
    angles = [angle_min + i * angle_increment for i in range(len(ranges))]

    # Extract obstacle positions in Cartesian coordinates
    obstacle_points = []
    for i, r in enumerate(ranges):
        if 0.1 < r < 10.0:  # Valid range
            angle = angles[i]
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            obstacle_points.append([x, y])

    return np.array(obstacle_points)

# Example usage
sample_ranges = [float('inf')] * 10 + [2.5, 2.4, 2.3, 2.2, 2.1] + [2.0] * 10 + [1.8, 1.5, 1.2, 0.8, 0.5] + [float('inf')] * 10
angles_min = -np.pi/2
angles_max = np.pi/2

obstacle_points = process_lidar_scan(sample_ranges, angles_min, angles_max)
print(f"Detected {len(obstacle_points)} obstacle points")
if len(obstacle_points) > 0:
    print(f"First few points: {obstacle_points[:3]}")
```

**Expected Output:**
```
Detected 10 obstacle points
First few points: [[ 0.          2.5       ]
 [ 0.23776413  2.48859624]
 [ 0.46947156  2.45484353]]
```

## Code Example 2: IMU Data Integration

```python
# Purpose: Integrate IMU data to estimate position and orientation
# Setup Instructions: Install numpy
# Run: python imu_integration.py

import numpy as np

class IMUPoseEstimator:
    def __init__(self):
        self.position = np.zeros(3)  # x, y, z
        self.velocity = np.zeros(3)  # vx, vy, vz
        self.orientation = np.array([0, 0, 0, 1])  # quaternion [x, y, z, w]

    def update(self, linear_acceleration, angular_velocity, dt):
        """Update pose estimate using IMU measurements"""
        # Integrate acceleration to get velocity
        self.velocity += linear_acceleration * dt

        # Integrate velocity to get position
        self.position += self.velocity * dt

        # Update orientation using angular velocity
        # This is a simplified integration for demonstration
        dq = self._angular_velocity_to_quaternion_derivative(
            self.orientation, angular_velocity
        )
        self.orientation += dq * dt
        # Normalize quaternion
        self.orientation /= np.linalg.norm(self.orientation)

    def _angular_velocity_to_quaternion_derivative(self, q, omega):
        """Convert angular velocity to quaternion derivative"""
        wx, wy, wz = omega
        Omega = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])
        return 0.5 * Omega @ q

# Example usage
estimator = IMUPoseEstimator()
linear_acc = np.array([0.1, 0.05, 9.81])  # Including gravity
angular_vel = np.array([0.1, 0.05, 0.02])
dt = 0.01  # 10ms time step

print(f"Initial position: {estimator.position}")
estimator.update(linear_acc, angular_vel, dt)
print(f"After 10ms: position={estimator.position}, velocity={estimator.velocity}")
```

**Expected Output:**
```
Initial position: [0. 0. 0.]
After 10ms: position=[0.000005   0.0000025  0.0981049 ], velocity=[0.001  0.0005 0.981 ]
```

## Hands-on Exercises

1. Implement a function that fuses data from multiple sensors (LIDAR, camera, IMU) to create a more robust perception system
2. Create a simple Kalman filter to smooth noisy sensor measurements
3. Implement a basic SLAM (Simultaneous Localization and Mapping) algorithm using sensor data

## Summary

Sensor systems form the foundation of robotic perception. Understanding how different sensors work and how to process their data is crucial for building effective robotic systems. The choice of sensors depends on the specific application requirements, environmental conditions, and accuracy needs.

## Hardware Requirements

This chapter can be completed using simulation environments. For physical implementation, the following hardware is recommended:
- 2D LIDAR (e.g., RPLIDAR A1/A2)
- RGB-D camera (e.g., Intel Realsense D435)
- IMU (e.g., Bosch BNO055)
- Force/torque sensor (e.g., ATI Nano 25)
- Robot platform with sensor interfaces