---
sidebar_position: 1
title: "ROS 2 Architecture: Nodes, Topics, Services, Actions"
---

# ROS 2 Architecture: Nodes, Topics, Services, Actions

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the core concepts of ROS 2 architecture (nodes, topics, services, actions)
- Implement basic ROS 2 nodes for communication
- Compare the differences between topics, services, and actions
- Design appropriate communication patterns for different robotic tasks

## Prerequisites

- Basic understanding of distributed systems concepts
- Familiarity with Python programming
- Understanding of publisher-subscriber and client-server patterns

## Theory

ROS 2 (Robot Operating System 2) provides a flexible framework for developing robot applications. It implements a distributed peer-to-peer architecture that enables different components of a robot system to communicate and coordinate with each other.

### Core Architecture Concepts

**Nodes**: A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. Multiple nodes are typically aggregated together to form a complete robot application.

**Topics**: Topics enable asynchronous communication between nodes using a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics. This is ideal for streaming data like sensor readings.

**Services**: Services enable synchronous request-response communication between nodes. A client sends a request to a service, and the service sends back a response. This is ideal for operations that require a definitive result.

**Actions**: Actions enable asynchronous goal-request-result communication with feedback. Actions are ideal for long-running tasks where you need to monitor progress and potentially cancel the operation.

### DDS Middleware

ROS 2 uses Data Distribution Service (DDS) as its underlying middleware. DDS provides discovery, serialization, and transport mechanisms that enable nodes to communicate transparently across different platforms and programming languages.

## Code Example 1: Basic ROS 2 Publisher Node

```python
# Purpose: Create a simple publisher node that publishes sensor data
# Setup Instructions: ROS 2 Humble Hawksbill installed
# Run: ros2 run my_package sensor_publisher.py (after proper setup)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Sensor reading {self.i}: {random.uniform(0, 100):.2f}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output:**
```
[INFO] [1620123456.789] [sensor_publisher]: Publishing: "Sensor reading 0: 42.67"
[INFO] [1620123457.289] [sensor_publisher]: Publishing: "Sensor reading 1: 78.23"
[INFO] [1620123457.789] [sensor_publisher]: Publishing: "Sensor reading 2: 15.89"
```

## Code Example 2: Basic ROS 2 Subscriber Node

```python
# Purpose: Create a subscriber node that receives sensor data
# Setup Instructions: ROS 2 Humble Hawksbill installed
# Run: ros2 run my_package sensor_subscriber.py (after proper setup)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output:**
```
[INFO] [1620123456.789] [sensor_subscriber]: I heard: "Sensor reading 0: 42.67"
[INFO] [1620123457.289] [sensor_subscriber]: I heard: "Sensor reading 1: 78.23"
[INFO] [1620123457.789] [sensor_subscriber]: I heard: "Sensor reading 2: 15.89"
```

## Hands-on Exercises

1. Create a ROS 2 service server that performs a mathematical calculation (e.g., computing distance between two points) and a client that calls this service
2. Implement a ROS 2 action server that simulates moving a robot to a goal position with feedback on progress
3. Design a multi-node ROS 2 system that demonstrates the use of topics, services, and actions together for a simple robot task

## Summary

ROS 2 provides a robust architecture for building distributed robotic systems. Understanding the differences between nodes, topics, services, and actions is crucial for designing effective robot applications. The choice of communication pattern depends on the timing requirements and nature of the interaction between components.

## Hardware Requirements

This chapter can be completed using simulation environments. For physical implementation, the following hardware is recommended:
- Computer with ROS 2 installed (Ubuntu 22.04 with ROS 2 Humble Hawksbill)
- Network connectivity for multi-machine setups
- Robot hardware with ROS 2 drivers (optional for basic exercises)