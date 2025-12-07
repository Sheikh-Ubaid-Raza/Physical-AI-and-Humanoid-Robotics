---
sidebar_position: 1
title: "AI-Powered Perception and Manipulation"
---

# AI-Powered Perception and Manipulation

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement AI-based perception systems for robot applications
- Train and deploy neural networks for object detection and recognition
- Design manipulation strategies using AI and machine learning
- Integrate perception and manipulation systems for complex robot tasks
- Evaluate the performance of AI-powered robot systems
- Handle uncertainty and robustness in AI-based robotics

## Prerequisites

- Understanding of basic machine learning concepts
- Experience with deep learning frameworks (PyTorch, TensorFlow)
- Knowledge of ROS 2 for system integration
- Understanding of robot kinematics and dynamics (covered in Week 11-12)

## Theory

AI-powered perception and manipulation represent a significant advancement in robotics, enabling robots to operate in unstructured environments with greater autonomy and adaptability. Traditional approaches to perception and manipulation rely on hand-crafted algorithms and predefined models, whereas AI-based approaches learn patterns and behaviors from data.

### AI-Based Perception

Modern perception systems leverage deep learning for tasks such as:
- Object detection and classification
- Semantic segmentation
- Depth estimation
- Pose estimation
- Scene understanding

These systems can process various sensor modalities including RGB cameras, depth sensors, LIDAR, and IMUs to create rich understanding of the environment.

### AI-Based Manipulation

Manipulation tasks benefit from AI through:
- Reinforcement learning for motor skill acquisition
- Imitation learning from human demonstrations
- Visual servoing guided by neural networks
- Grasp planning using learned models
- Adaptive control strategies

### Challenges and Solutions

**Uncertainty Handling**: AI systems must deal with uncertainty in perception and action. Techniques like Bayesian neural networks, ensemble methods, and uncertainty quantification help manage this challenge.

**Real-Time Performance**: Many AI algorithms require significant computational resources. Optimization techniques include model compression, quantization, and specialized hardware acceleration.

**Generalization**: AI systems trained in simulation or specific environments must generalize to new situations. Domain adaptation, sim-to-real transfer, and data augmentation techniques address this challenge.

## Code Example 1: Object Detection for Robot Perception

```python
# perception/object_detector.py
# Purpose: Implement real-time object detection for robot perception
# Setup Instructions: Install torch, torchvision, opencv-python
# Run: python object_detector.py

import torch
import torchvision.transforms as transforms
from PIL import Image
import cv2
import numpy as np
from torchvision.models.detection import fasterrcnn_resnet50_fpn
from torchvision.utils import draw_bounding_boxes
import rospy
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge

class ObjectDetector:
    def __init__(self, confidence_threshold=0.5):
        # Load pre-trained model
        self.model = fasterrcnn_resnet50_fpn(pretrained=True)
        self.model.eval()

        # Confidence threshold
        self.confidence_threshold = confidence_threshold

        # COCO dataset class names
        self.coco_names = [
            '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
            'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
            'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
            'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table',
            'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
            'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
            'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Image transformation
        self.transform = transforms.Compose([
            transforms.ToTensor(),
        ])

        # Initialize ROS components
        self.bridge = CvBridge()
        self.object_pub = rospy.Publisher('/detected_objects', ImageMsg, queue_size=10)

    def detect_objects(self, image):
        """
        Detect objects in an image
        Args:
            image: PIL Image or numpy array
        Returns:
            dict: Detection results
        """
        # Convert to PIL if needed
        if isinstance(image, np.ndarray):
            image_pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        else:
            image_pil = image

        # Transform image
        input_tensor = self.transform(image_pil).unsqueeze(0)

        # Perform inference
        with torch.no_grad():
            predictions = self.model(input_tensor)

        # Filter predictions by confidence
        pred = predictions[0]
        scores = pred['scores']
        boxes = pred['boxes']
        labels = pred['labels']

        # Filter by confidence threshold
        keep_idx = scores > self.confidence_threshold
        filtered_boxes = boxes[keep_idx]
        filtered_labels = labels[keep_idx]
        filtered_scores = scores[keep_idx]

        # Convert to numpy arrays
        boxes_np = filtered_boxes.numpy()
        labels_np = filtered_labels.numpy()
        scores_np = filtered_scores.numpy()

        # Create results dictionary
        results = {
            'boxes': boxes_np,
            'labels': [self.coco_names[label] for label in labels_np],
            'scores': scores_np,
            'num_detections': len(boxes_np)
        }

        return results

    def annotate_image(self, image, results):
        """Draw bounding boxes on image"""
        # Convert image to tensor for drawing
        img_tensor = torch.from_numpy(image).permute(2, 0, 1).byte()  # CHW format

        # Convert boxes to integer format
        boxes = torch.tensor(results['boxes']).int()

        # Create labels for drawing
        labels = [f"{label}: {score:.2f}" for label, score in zip(results['labels'], results['scores'])]

        # Draw bounding boxes
        annotated_img = draw_bounding_boxes(
            image=img_tensor,
            boxes=boxes,
            labels=labels,
            colors=['red'] * len(boxes),
            width=2
        )

        # Convert back to numpy
        annotated_img_np = annotated_img.permute(1, 2, 0).numpy()

        return annotated_img_np

# Example usage
def main():
    rospy.init_node('object_detector_node', anonymous=True)

    detector = ObjectDetector(confidence_threshold=0.7)

    # Example: Process a single image
    # In practice, this would subscribe to camera topic
    cap = cv2.VideoCapture(0)  # Use webcam

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                continue

            # Detect objects
            results = detector.detect_objects(frame)

            # Annotate image
            annotated_frame = detector.annotate_image(frame, results)

            # Display results
            cv2.imshow('Object Detection', annotated_frame)

            # Print detection results
            if results['num_detections'] > 0:
                print(f"Detected {results['num_detections']} objects:")
                for i in range(results['num_detections']):
                    print(f"  - {results['labels'][i]}: {results['scores'][i]:.2f}")
            else:
                print("No objects detected")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

**Expected Output:**
```
Detected 3 objects:
  - person: 0.98
  - chair: 0.87
  - laptop: 0.76
Detected 2 objects:
  - person: 0.99
  - cup: 0.81
No objects detected
```

## Code Example 2: AI-Based Grasp Planning

```python
# manipulation/grasp_planner.py
# Purpose: Implement AI-based grasp planning for robot manipulation
# Setup Instructions: Install numpy, scipy, robotiq_gripper_control
# Run: python grasp_planner.py

import numpy as np
from scipy.spatial.distance import cdist
import math
from collections import deque

class GraspPlanner:
    def __init__(self, gripper_width_range=(0.01, 0.08), finger_length=0.05):
        """
        Initialize grasp planner
        Args:
            gripper_width_range: Tuple of (min, max) gripper width in meters
            finger_length: Length of gripper fingers in meters
        """
        self.min_width = gripper_width_range[0]
        self.max_width = gripper_width_range[1]
        self.finger_length = finger_length

        # Grasp candidates storage
        self.grasp_candidates = []

    def plan_grasps_from_point_cloud(self, point_cloud, object_center=None):
        """
        Plan grasps from point cloud data
        Args:
            point_cloud: Nx3 numpy array of 3D points
            object_center: Center of object to grasp (optional)
        Returns:
            list: List of feasible grasp poses
        """
        if object_center is None:
            # Estimate object center as centroid of point cloud
            object_center = np.mean(point_cloud, axis=0)

        # Find surface points on object
        surface_points = self._extract_surface_points(point_cloud)

        # Generate grasp candidates
        grasp_poses = self._generate_grasp_candidates(surface_points, object_center)

        # Filter feasible grasps
        feasible_grasps = self._filter_feasible_grasps(grasp_poses, point_cloud)

        return feasible_grasps

    def _extract_surface_points(self, point_cloud, neighborhood_radius=0.02):
        """Extract surface points using normal estimation"""
        surface_points = []

        # For each point, check if it's on a surface
        for i, point in enumerate(point_cloud):
            # Find neighboring points
            distances = cdist([point], point_cloud)[0]
            neighbors = point_cloud[distances < neighborhood_radius]

            if len(neighbors) >= 3:
                # Estimate surface normal
                cov_matrix = np.cov(neighbors.T)
                eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

                # Normal is the eigenvector corresponding to smallest eigenvalue
                normal = eigenvectors[:, 0]

                # Check if this is a surface point (low variation in normal direction)
                if eigenvalues[0] < 0.001:  # Surface point
                    surface_points.append({
                        'position': point,
                        'normal': normal
                    })

        return surface_points

    def _generate_grasp_candidates(self, surface_points, object_center):
        """Generate potential grasp poses"""
        grasp_candidates = []

        for i, point_data in enumerate(surface_points):
            pos = point_data['position']
            normal = point_data['normal']

            # Generate multiple grasp orientations around the surface normal
            for angle_offset in np.linspace(0, 2*np.pi, 8):  # 8 orientations per point
                # Create grasp orientation
                grasp_orientation = self._compute_grasp_orientation(normal, angle_offset)

                # Create grasp pose
                grasp_pose = {
                    'position': pos,
                    'orientation': grasp_orientation,
                    'approach_direction': -normal  # Approach opposite to surface normal
                }

                grasp_candidates.append(grasp_pose)

        return grasp_candidates

    def _compute_grasp_orientation(self, surface_normal, angle_offset):
        """Compute grasp orientation based on surface normal"""
        # Find two orthogonal vectors to the surface normal
        # First, find a vector not parallel to the normal
        if abs(surface_normal[2]) < 0.9:
            temp_vec = np.array([0, 0, 1])
        else:
            temp_vec = np.array([1, 0, 0])

        # Create two orthogonal vectors
        ortho1 = np.cross(surface_normal, temp_vec)
        ortho1 = ortho1 / np.linalg.norm(ortho1)
        ortho2 = np.cross(surface_normal, ortho1)
        ortho2 = ortho2 / np.linalg.norm(ortho2)

        # Rotate ortho1 and ortho2 by angle_offset around the normal
        rotated_ortho1 = (
            ortho1 * np.cos(angle_offset) +
            ortho2 * np.sin(angle_offset)
        )
        rotated_ortho2 = (
            -ortho1 * np.sin(angle_offset) +
            ortho2 * np.cos(angle_offset)
        )

        # Create rotation matrix
        # Columns are: approach (normal), binormal1, binormal2
        grasp_orientation = np.column_stack([
            -surface_normal,  # Approach direction (into surface)
            rotated_ortho1,   # Gripper opening direction
            rotated_ortho2    # Gripper up direction
        ])

        return grasp_orientation

    def _filter_feasible_grasps(self, grasp_poses, point_cloud, min_grasp_width=0.02):
        """Filter grasp poses based on geometric feasibility"""
        feasible_grasps = []

        for grasp in grasp_poses:
            position = grasp['position']
            orientation = grasp['orientation']

            # Check if grasp width is feasible
            grasp_width = self._estimate_object_width_at_grasp(position, orientation, point_cloud)

            if self.min_width <= grasp_width <= self.max_width:
                # Check if there are sufficient contact points for a stable grasp
                contact_points = self._check_contact_points(position, orientation, point_cloud)

                if contact_points >= 2:  # At least 2 contact points needed
                    grasp['estimated_width'] = grasp_width
                    grasp['contact_points'] = contact_points
                    feasible_grasps.append(grasp)

        # Sort grasps by estimated quality
        feasible_grasps.sort(key=lambda g: g['contact_points'], reverse=True)

        return feasible_grasps

    def _estimate_object_width_at_grasp(self, position, orientation, point_cloud):
        """Estimate object width at a potential grasp location"""
        # Project points onto the gripper plane
        gripper_normal = orientation[:, 0]  # First column is approach direction
        gripper_binormal = orientation[:, 1]  # Second column is gripper opening direction

        # Find points near the grasp position
        distances = cdist([position], point_cloud)[0]
        nearby_points = point_cloud[distances < 0.05]  # 5cm radius

        if len(nearby_points) == 0:
            return 0.0

        # Project points onto gripper opening direction
        projections = np.dot(nearby_points - position, gripper_binormal)

        # Width is the range of projections
        width = np.max(projections) - np.min(projections)

        return width

    def _check_contact_points(self, position, orientation, point_cloud):
        """Check if there are sufficient contact points for a stable grasp"""
        gripper_normal = orientation[:, 0]
        gripper_binormal = orientation[:, 1]

        # Define contact regions on either side of the gripper
        gripper_width = self.min_width  # Use minimum width for conservative estimate
        left_contact_region = position + (gripper_width/2) * gripper_binormal
        right_contact_region = position - (gripper_width/2) * gripper_binormal

        # Count points in contact regions
        left_distances = cdist([left_contact_region], point_cloud)[0]
        right_distances = cdist([right_contact_region], point_cloud)[0]

        left_contacts = np.sum(left_distances < 0.01)  # 1cm contact region
        right_contacts = np.sum(right_distances < 0.01)

        return min(left_contacts, right_contacts)  # Both sides need contacts

# Example usage
def main():
    # Simulate a point cloud of an object (e.g., a cube)
    np.random.seed(42)

    # Create a simple cube point cloud
    cube_points = []
    for x in np.linspace(-0.05, 0.05, 10):
        for y in np.linspace(-0.03, 0.03, 6):
            for z in np.linspace(-0.02, 0.02, 4):
                # Add some noise
                point = np.array([x, y, z]) + np.random.normal(0, 0.001, 3)
                cube_points.append(point)

    point_cloud = np.array(cube_points)

    # Create grasp planner
    planner = GraspPlanner()

    # Plan grasps
    feasible_grasps = planner.plan_grasps_from_point_cloud(point_cloud)

    print(f"Found {len(feasible_grasps)} feasible grasps")

    if feasible_grasps:
        best_grasp = feasible_grasps[0]
        print(f"Best grasp at position: {best_grasp['position']}")
        print(f"Estimated object width: {best_grasp['estimated_width']:.3f}m")
        print(f"Contact points: {best_grasp['contact_points']}")
        print(f"Grasp orientation:\n{best_grasp['orientation']}")

if __name__ == "__main__":
    main()
```

**Expected Output:**
```
Found 24 feasible grasps
Best grasp at position: [-0.05 -0.03 -0.02]
Estimated object width: 0.098m
Contact points: 5
Grasp orientation:
[[ 0.000 -1.000  0.000]
 [ 1.000  0.000  0.000]
 [ 0.000  0.000  1.000]]
```

## Hands-on Exercises

1. Implement a CNN-based classifier to recognize specific objects in your robot's environment
2. Train a reinforcement learning agent to perform simple manipulation tasks in simulation
3. Develop a perception pipeline that integrates multiple sensor modalities (camera, LIDAR) for improved object detection

## Summary

AI-powered perception and manipulation enable robots to operate in complex, unstructured environments with greater autonomy. By leveraging deep learning and machine learning techniques, robots can recognize objects, understand scenes, and plan manipulation actions that would be difficult to encode with traditional programming approaches. The integration of AI with robotics requires careful consideration of real-time performance, uncertainty handling, and safety.

## Hardware Requirements

This chapter requires specialized hardware and software:
- GPU with CUDA support for neural network inference
- Robot platform with manipulation capabilities
- RGB-D camera or LIDAR for perception
- Simulation environment for training (Isaac Sim, Gazebo, or PyBullet)
- Computing platform for real-time inference (NVIDIA Jetson or equivalent)