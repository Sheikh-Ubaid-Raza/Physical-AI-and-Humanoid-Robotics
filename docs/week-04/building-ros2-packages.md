---
sidebar_position: 1
title: "Building ROS 2 Packages with Python (rclpy)"
---

# Building ROS 2 Packages with Python (rclpy)

## Learning Objectives

By the end of this chapter, you will be able to:
- Create and structure ROS 2 packages using Python
- Implement nodes using the rclpy library
- Configure package dependencies and build files
- Build and run custom ROS 2 packages
- Debug common issues in Python-based ROS 2 packages

## Prerequisites

- Understanding of ROS 2 architecture (covered in Week 3)
- Proficiency in Python programming
- Basic familiarity with package managers and build systems

## Theory

ROS 2 packages are the fundamental building units of ROS 2 software. A package contains libraries, executables, scripts, or other artifacts. The rclpy library provides a Python client library for ROS 2, allowing developers to create ROS 2 nodes using Python.

### Package Structure

A typical ROS 2 Python package follows this structure:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration (even for pure Python packages)
├── package.xml             # Package manifest
├── setup.cfg               # Installation configuration
├── setup.py                # Python package setup
├── my_robot_package/       # Python module
│   ├── __init__.py
│   └── my_nodes.py
└── test/                   # Test files
```

### Key Components

**package.xml**: Contains package metadata including name, version, description, authors, maintainers, licenses, dependencies, and export tags.

**setup.py**: Standard Python packaging file that specifies how the package should be built and installed. For ROS 2 packages, this includes information about entry points and data files.

**rclpy**: The Python client library for ROS 2 that provides Python bindings to the ROS 2 client library (rcl). It allows creating nodes, publishers, subscribers, services, and actions.

## Code Example 1: Creating a Basic ROS 2 Package Structure

```python
# setup.py
# Purpose: Define the Python package structure for ROS 2
# Setup Instructions: Place in root of package directory
# Run: python setup.py install (or colcon build in workspace)

from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A simple ROS 2 package for robot control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_package.simple_publisher:main',
            'simple_subscriber = my_robot_package.simple_subscriber:main',
        ],
    },
)
```

**Expected Output:**
```
# After running colcon build:
Starting >>> my_robot_package
Finished <<< my_robot_package [2.35s]

# Package can be sourced and executables run:
source install/setup.bash
ros2 run my_robot_package simple_publisher
```

## Code Example 2: Python ROS 2 Node Implementation

```python
# my_robot_package/simple_publisher.py
# Purpose: Implement a simple publisher node using rclpy
# Setup Instructions: Part of ROS 2 package structure
# Run: ros2 run my_robot_package simple_publisher

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create a publisher
        self.publisher = self.create_publisher(
            String,
            'chatter',
            10  # Queue size
        )

        # Create a timer for periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.counter = 0
        self.get_logger().info('Simple Publisher has started')

    def timer_callback(self):
        """Callback function for the timer"""
        msg = String()
        msg.data = f'Hello ROS 2 World: {self.counter}'

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

        self.counter += 1

def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    try:
        publisher_node = SimplePublisher()

        # Spin the node to execute callbacks
        rclpy.spin(publisher_node)

    except KeyboardInterrupt:
        print("\nShutting down node...")

    finally:
        # Cleanup
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output:**
```
[INFO] [1620123456.789] [simple_publisher]: Simple Publisher has started
[INFO] [1620123457.289] [simple_publisher]: Published: Hello ROS 2 World: 0
[INFO] [1620123457.789] [simple_publisher]: Published: Hello ROS 2 World: 1
[INFO] [1620123458.289] [simple_publisher]: Published: Hello ROS 2 World: 2
```

## Hands-on Exercises

1. Create a ROS 2 package that implements a temperature monitoring system with a publisher node and a subscriber node
2. Build a ROS 2 package that provides a service to convert temperatures between Celsius and Fahrenheit
3. Develop a ROS 2 package with multiple nodes that coordinate to perform a simple robot task

## Summary

Building ROS 2 packages with Python provides a powerful way to develop robot applications. The rclpy library offers a clean Pythonic interface to ROS 2 concepts. Proper package structure and configuration are essential for creating reusable and maintainable robot software.

## Hardware Requirements

This chapter can be completed using simulation environments. For physical implementation, the following hardware is recommended:
- Computer with ROS 2 installed (Ubuntu 22.04 with ROS 2 Humble Hawksbill)
- Python 3.8 or higher
- Appropriate robot hardware with ROS 2 compatibility (optional for basic exercises)