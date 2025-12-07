---
sidebar_position: 1
title: "Unity for Robot Visualization"
---

# Unity for Robot Visualization

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up Unity for robot visualization and simulation
- Import and configure robot models in Unity
- Create realistic environments for robot visualization
- Implement sensor simulation in Unity
- Connect Unity visualization to ROS 2 systems
- Design user interfaces for robot monitoring and control

## Prerequisites

- Basic understanding of Unity game engine
- Knowledge of robot modeling and URDF (covered in Week 5)
- Experience with ROS 2 systems (covered in Weeks 3-4)

## Theory

Unity is a powerful 3D development platform that can be used for robot visualization, simulation, and teleoperation interfaces. While Gazebo is more focused on physics simulation, Unity excels in creating visually appealing and interactive environments for robot visualization.

### Unity Robotics Hub

The Unity Robotics Hub provides tools and samples to connect Unity with ROS 2, enabling bidirectional communication between Unity and ROS 2 systems. This allows for real-time visualization of robot states and sensor data from ROS 2 systems.

### URDF Importer

Unity provides a URDF Importer package that allows importing robot models defined in URDF format. This enables leveraging existing robot models in Unity scenes.

### ROS TCP Connector

The ROS TCP Connector package enables communication between Unity and ROS 2 systems over TCP/IP. This allows Unity to subscribe to ROS 2 topics and publish messages to ROS 2 topics.

### Robot Simulation vs Visualization

In Unity, you can focus on either visualization (showing robot state and sensor data from ROS 2) or simulation (performing physics simulation and sending data to ROS 2). For visualization, Unity typically connects to a running ROS 2 system and displays the robot's state in real-time.

## Code Example 1: Unity Robot Controller Script

```csharp
// RobotController.cs
// Purpose: Unity script to control robot visualization based on ROS 2 data
// Setup Instructions: Attach to robot GameObject in Unity
// Run: Unity Play Mode or Build

using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    [SerializeField]
    private string robotName = "my_robot";

    [SerializeField]
    private float maxLinearVelocity = 1.0f;
    [SerializeField]
    private float maxAngularVelocity = 1.0f;

    private ROSConnection ros;
    private TwistMsg cmdVelMsg;
    private OdometryMsg odomMsg;

    // Subscribe to topics
    private string cmdVelTopic;
    private string odomTopic;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Set up topic names
        cmdVelTopic = "/" + robotName + "/cmd_vel";
        odomTopic = "/" + robotName + "/odom";

        // Subscribe to odometry topic
        ros.Subscribe<OdometryMsg>(odomTopic, OdomCallback);

        // Initialize command message
        cmdVelMsg = new TwistMsg();
        cmdVelMsg.linear = new Vector3Msg();
        cmdVelMsg.angular = new Vector3Msg();
    }

    void OdomCallback(OdometryMsg msg)
    {
        // Update robot position based on odometry
        Vector3 newPosition = new Vector3(
            (float)msg.pose.pose.position.x,
            (float)msg.pose.pose.position.z, // Unity Y is up, ROS Z is up
            (float)msg.pose.pose.position.y
        );

        Quaternion newRotation = new Quaternion(
            (float)msg.pose.pose.orientation.x,
            (float)msg.pose.pose.orientation.z, // Unity Y is up, ROS Z is up
            (float)msg.pose.pose.orientation.y,
            (float)msg.pose.pose.orientation.w
        );

        transform.position = newPosition;
        transform.rotation = newRotation;
    }

    public void SendCmdVel(float linearX, float angularZ)
    {
        // Clamp velocities
        cmdVelMsg.linear.x = Mathf.Clamp(linearX, -maxLinearVelocity, maxLinearVelocity);
        cmdVelMsg.angular.z = Mathf.Clamp(angularZ, -maxAngularVelocity, maxAngularVelocity);

        // Publish command
        ros.Publish(cmdVelTopic, cmdVelMsg);
    }

    void Update()
    {
        // Handle input for teleoperation
        float linearInput = Input.GetAxis("Vertical");
        float angularInput = Input.GetAxis("Horizontal");

        if (Input.GetKey(KeyCode.Space))
        {
            SendCmdVel(linearInput * maxLinearVelocity, angularInput * maxAngularVelocity);
        }
    }
}
```

**Expected Output:**
```
// In Unity Console:
Connected to ROS bridge
Subscribed to /my_robot/odom
Publishing to /my_robot/cmd_vel
Robot position updated: (2.5, 0.1, 1.8)
Robot rotation updated: (0, 0.707, 0, 0.707)
```

## Code Example 2: Unity Sensor Visualization Script

```csharp
// LaserScanVisualizer.cs
// Purpose: Visualize LIDAR data in Unity
// Setup Instructions: Attach to GameObject to display laser scan
// Run: Unity Play Mode

using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class LaserScanVisualizer : MonoBehaviour
{
    [SerializeField]
    private string laserTopic = "/scan";
    [SerializeField]
    private GameObject laserPointPrefab; // Prefab for individual laser points
    [SerializeField]
    private Color laserColor = Color.red;
    [SerializeField]
    private float maxRange = 10.0f;

    private ROSConnection ros;
    private GameObject[] laserPoints;
    private int maxPoints = 1080; // Typical LIDAR resolution

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>(laserTopic, LaserScanCallback);

        // Create laser point objects
        laserPoints = new GameObject[maxPoints];
        for (int i = 0; i < maxPoints; i++)
        {
            laserPoints[i] = Instantiate(laserPointPrefab, transform);
            laserPoints[i].SetActive(false);
        }
    }

    void LaserScanCallback(LaserScanMsg scan)
    {
        // Calculate angle increment
        float angleIncrement = (float)scan.angle_increment;
        float currentAngle = (float)scan.angle_min;

        // Process each range measurement
        for (int i = 0; i < scan.ranges.Length && i < maxPoints; i++)
        {
            float range = scan.ranges[i];

            if (!float.IsNaN(range) && !float.IsInfinity(range) && range <= maxRange)
            {
                // Calculate position in 2D space
                float x = range * Mathf.Cos(currentAngle);
                float y = range * Mathf.Sin(currentAngle);

                // Position the laser point
                laserPoints[i].transform.localPosition = new Vector3(x, 0.1f, y); // Y is slightly above ground
                laserPoints[i].GetComponent<Renderer>().material.color = laserColor;
                laserPoints[i].SetActive(true);
            }
            else
            {
                laserPoints[i].SetActive(false);
            }

            currentAngle += angleIncrement;
        }

        // Deactivate remaining points
        for (int i = scan.ranges.Length; i < maxPoints; i++)
        {
            laserPoints[i].SetActive(false);
        }
    }
}
```

**Expected Output:**
```
// Laser scan visualization shows:
// - Points representing obstacles in the environment
// - Real-time updating as robot moves
// - Proper scaling and coloring of laser points
// - Clean visualization without artifacts
```

## Hands-on Exercises

1. Create a Unity scene with a robot model imported from URDF and implement basic movement controls
2. Implement a camera system in Unity that follows the robot and provides multiple viewpoints
3. Design a user interface in Unity that displays robot sensor data and allows teleoperation

## Summary

Unity provides powerful visualization capabilities for robotics applications, complementing traditional simulation tools like Gazebo. The ability to create visually appealing and interactive environments makes Unity ideal for robot teleoperation interfaces, training applications, and presentation of robot capabilities. Integration with ROS 2 enables real-time visualization of robot systems.

## Hardware Requirements

This chapter can be completed using simulation environments. For physical implementation, the following hardware is recommended:
- Computer with Unity Hub and Unity Editor (2021.3 LTS or newer)
- Unity Robotics packages installed via Package Manager
- Robot hardware for connecting to ROS 2 system (optional for basic exercises)
- VR headset for immersive visualization (optional)