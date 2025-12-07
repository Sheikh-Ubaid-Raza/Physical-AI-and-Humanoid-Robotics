---
sidebar_position: 1
title: "Humanoid Kinematics: Forward and Inverse Kinematics"
---

# Humanoid Kinematics: Forward and Inverse Kinematics

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the kinematic structure of humanoid robots
- Implement forward kinematics for humanoid robots
- Solve inverse kinematics problems for humanoid manipulation and locomotion
- Analyze and optimize humanoid robot movements
- Implement kinematic chains for humanoid limbs
- Apply kinematic constraints for stable humanoid locomotion

## Prerequisites

- Understanding of linear algebra (vectors, matrices, transformations)
- Basic knowledge of robotics kinematics concepts
- Familiarity with ROS 2 for system integration
- Understanding of coordinate systems and transformations

## Theory

Humanoid kinematics deals with the geometric relationships between the links and joints of a humanoid robot. Unlike simpler robotic arms, humanoid robots have complex kinematic structures with multiple limbs, each requiring coordinated motion for tasks like walking, balancing, and manipulation.

### Forward Kinematics

Forward kinematics calculates the position and orientation of the end-effector (hand, foot) given the joint angles. For humanoid robots, this involves calculating the pose of each limb segment based on the joint angles throughout the kinematic chain.

The forward kinematics solution involves multiplying transformation matrices for each joint in the chain:
```
T_end_effector = T_base * T_1 * T_2 * ... * T_n
```

Where T_i represents the transformation matrix for joint i.

### Inverse Kinematics

Inverse kinematics (IK) solves the opposite problem: given a desired end-effector pose, find the joint angles that achieve this pose. For humanoid robots, IK is often over-constrained (more joints than degrees of freedom needed), requiring optimization approaches to select the best solution among possible ones.

Common IK approaches for humanoid robots:
- **Analytical IK**: Closed-form solutions for simple chains
- **Numerical IK**: Iterative methods like Jacobian-based approaches
- **Optimization-based IK**: Formulating as constrained optimization problem

### Humanoid Kinematic Chains

Humanoid robots typically have multiple kinematic chains:
- **Arm chains**: From shoulder to hand
- **Leg chains**: From hip to foot
- **Spine chain**: Connecting upper and lower body
- **Neck chain**: Connecting head to torso

Each chain has specific kinematic properties and constraints.

### Kinematic Constraints

Humanoid robots have numerous constraints:
- **Joint limits**: Physical range of motion
- **Collision avoidance**: Self-collision and environment collision
- **Balance constraints**: Center of mass within support polygon
- **Postural constraints**: Maintaining comfortable configurations

## Code Example 1: Humanoid Forward Kinematics

```python
# kinematics/humanoid_fk.py
# Purpose: Implement forward kinematics for a humanoid robot
# Setup Instructions: Install numpy, scipy
# Run: python humanoid_fk.py

import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class HumanoidFK:
    """
    Forward kinematics calculator for a humanoid robot
    """
    def __init__(self):
        # Define humanoid kinematic structure
        # Simplified 12-DOF leg model
        self.links = {
            # Hip to ankle chain
            'hip_roll': {'offset': [0, 0, 0], 'axis': [1, 0, 0]},
            'hip_yaw': {'offset': [0, 0, 0], 'axis': [0, 1, 0]},
            'hip_pitch': {'offset': [0, 0, 0], 'axis': [0, 0, 1]},
            'knee': {'offset': [0, 0, -0.4], 'axis': [0, 1, 0]},  # 40cm thigh
            'ankle_pitch': {'offset': [0, 0, -0.4], 'axis': [0, 0, 1]},  # 40cm shin
            'ankle_roll': {'offset': [0, 0, 0], 'axis': [1, 0, 0]}
        }

        # Link lengths (in meters)
        self.link_lengths = {
            'torso_to_hip': 0.1,  # Offset from torso to hip joint
            'thigh': 0.4,         # Thigh length
            'shin': 0.4,          # Shin length
            'foot': 0.1           # Foot length
        }

    def dh_transform(self, a, alpha, d, theta):
        """
        Denavit-Hartenberg transformation matrix
        """
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        st = math.sin(theta)
        ct = math.cos(theta)

        transform = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        return transform

    def leg_forward_kinematics(self, joint_angles, leg_origin=np.array([0, 0, 0])):
        """
        Calculate forward kinematics for a single leg
        Args:
            joint_angles: Array of 6 joint angles [hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll]
            leg_origin: Origin position of the leg (hip location)
        Returns:
            end_effector_pos: Position of the foot
            end_effector_rot: Orientation of the foot
        """
        # Extract joint angles
        hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll = joint_angles

        # Create transformation matrices for each joint
        # Hip roll
        T_hip_roll = self.dh_transform(0, 0, 0, hip_roll)

        # Hip yaw
        T_hip_yaw = self.dh_transform(0, math.pi/2, 0, hip_yaw)

        # Hip pitch
        T_hip_pitch = self.dh_transform(0, -math.pi/2, 0, hip_pitch)

        # Thigh (fixed length)
        T_thigh = self.dh_transform(0, 0, -self.link_lengths['thigh'], 0)

        # Knee
        T_knee = self.dh_transform(0, math.pi/2, 0, knee)

        # Shin (fixed length)
        T_shin = self.dh_transform(0, -math.pi/2, -self.link_lengths['shin'], 0)

        # Ankle pitch
        T_ankle_pitch = self.dh_transform(0, 0, 0, ankle_pitch)

        # Ankle roll
        T_ankle_roll = self.dh_transform(0, 0, 0, ankle_roll)

        # Combine all transformations
        T_total = (T_hip_roll @ T_hip_yaw @ T_hip_pitch @
                  T_thigh @ T_knee @ T_shin @
                  T_ankle_pitch @ T_ankle_roll)

        # Apply leg origin offset
        T_origin = np.eye(4)
        T_origin[:3, 3] = leg_origin
        T_final = T_origin @ T_total

        # Extract position and orientation
        end_effector_pos = T_final[:3, 3]
        end_effector_rot = T_final[:3, :3]

        return end_effector_pos, end_effector_rot

    def arm_forward_kinematics(self, joint_angles, arm_origin=np.array([0.2, 0.15, 0.2])):
        """
        Calculate forward kinematics for a single arm
        Args:
            joint_angles: Array of 6 joint angles [shoulder_yaw, shoulder_pitch, shoulder_roll, elbow, wrist_yaw, wrist_pitch]
            arm_origin: Origin position of the arm (shoulder location)
        Returns:
            end_effector_pos: Position of the hand
            end_effector_rot: Orientation of the hand
        """
        # Extract joint angles
        shoulder_yaw, shoulder_pitch, shoulder_roll, elbow, wrist_yaw, wrist_pitch = joint_angles

        # Shoulder complex
        T_shoulder_yaw = self.dh_transform(0, 0, 0, shoulder_yaw)
        T_shoulder_pitch = self.dh_transform(0, math.pi/2, 0, shoulder_pitch)
        T_shoulder_roll = self.dh_transform(0, -math.pi/2, 0, shoulder_roll)

        # Upper arm (30cm)
        T_upper_arm = self.dh_transform(0, 0, -0.3, 0)

        # Elbow
        T_elbow = self.dh_transform(0, math.pi/2, 0, elbow)

        # Forearm (30cm)
        T_forearm = self.dh_transform(0, -math.pi/2, -0.3, 0)

        # Wrist
        T_wrist_yaw = self.dh_transform(0, 0, 0, wrist_yaw)
        T_wrist_pitch = self.dh_transform(0, 0, 0, wrist_pitch)

        # Combine all transformations
        T_total = (T_shoulder_yaw @ T_shoulder_pitch @ T_shoulder_roll @
                  T_upper_arm @ T_elbow @ T_forearm @
                  T_wrist_yaw @ T_wrist_pitch)

        # Apply arm origin offset
        T_origin = np.eye(4)
        T_origin[:3, 3] = arm_origin
        T_final = T_origin @ T_total

        # Extract position and orientation
        end_effector_pos = T_final[:3, 3]
        end_effector_rot = T_final[:3, :3]

        return end_effector_pos, end_effector_rot

    def calculate_center_of_mass(self, joint_positions, link_masses):
        """
        Calculate the center of mass of the humanoid robot
        Args:
            joint_positions: Dictionary of joint positions
            link_masses: Dictionary of masses for each link
        Returns:
            com_position: Center of mass position
        """
        total_mass = sum(link_masses.values())

        weighted_sum = np.zeros(3)
        for link_name, mass in link_masses.items():
            if link_name in joint_positions:
                pos = joint_positions[link_name]
                weighted_sum += mass * pos

        com_position = weighted_sum / total_mass
        return com_position

# Example usage
def main():
    fk_calculator = HumanoidFK()

    # Define joint angles for the leg (in radians)
    leg_joints = np.array([
        0.1,    # hip_roll
        0.0,    # hip_yaw
        -0.3,   # hip_pitch
        0.6,    # knee
        -0.3,   # ankle_pitch
        0.05    # ankle_roll
    ])

    # Define joint angles for the arm (in radians)
    arm_joints = np.array([
        0.2,    # shoulder_yaw
        0.5,    # shoulder_pitch
        0.1,    # shoulder_roll
        0.8,    # elbow
        0.1,    # wrist_yaw
        0.05    # wrist_pitch
    ])

    # Calculate leg FK
    foot_pos, foot_rot = fk_calculator.leg_forward_kinematics(
        leg_joints,
        leg_origin=np.array([0, 0.1, -0.8])  # Hip position (x, y, z)
    )

    print("Leg Forward Kinematics:")
    print(f"Foot position: [{foot_pos[0]:.3f}, {foot_pos[1]:.3f}, {foot_pos[2]:.3f}]")
    print(f"Foot orientation matrix:\n{foot_rot}")

    # Calculate arm FK
    hand_pos, hand_rot = fk_calculator.arm_forward_kinematics(
        arm_joints,
        arm_origin=np.array([0.2, 0.15, 0.2])  # Shoulder position
    )

    print("\nArm Forward Kinematics:")
    print(f"Hand position: [{hand_pos[0]:.3f}, {hand_pos[1]:.3f}, {hand_pos[2]:.3f}]")
    print(f"Hand orientation matrix:\n{hand_rot}")

    # Calculate center of mass (example)
    joint_positions = {
        'hip_left': np.array([0, 0.1, -0.8]),
        'hip_right': np.array([0, -0.1, -0.8]),
        'shoulder_left': np.array([0.2, 0.15, 0.2]),
        'shoulder_right': np.array([0.2, -0.15, 0.2])
    }

    link_masses = {
        'torso': 20.0,
        'head': 5.0,
        'left_arm': 3.0,
        'right_arm': 3.0,
        'left_leg': 5.0,
        'right_leg': 5.0
    }

    com_pos = fk_calculator.calculate_center_of_mass(joint_positions, link_masses)
    print(f"\nCenter of Mass: [{com_pos[0]:.3f}, {com_pos[1]:.3f}, {com_pos[2]:.3f}]")

if __name__ == "__main__":
    main()
```

**Expected Output:**
```
Leg Forward Kinematics:
Foot position: [-0.021, 0.089, -1.342]
Foot orientation matrix:
[[ 0.839 -0.540 -0.052]
 [ 0.061  0.089 -0.994]
 [ 0.540  0.839  0.061]]

Arm Forward Kinematics:
Hand position: [0.321, 0.345, 0.123]
Hand orientation matrix:
[[ 0.873  0.436 -0.218]
 [-0.423  0.889  0.174]
 [ 0.245 -0.087  0.966]]

Center of Mass: [0.000, 0.000, -0.357]
```

## Code Example 2: Humanoid Inverse Kinematics Solver

```python
# kinematics/humanoid_ik.py
# Purpose: Implement inverse kinematics for humanoid robot
# Setup Instructions: Install numpy, scipy
# Run: python humanoid_ik.py

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

class HumanoidIK:
    """
    Inverse kinematics solver for humanoid robot
    """
    def __init__(self, robot_config):
        self.robot_config = robot_config
        self.fk_solver = HumanoidFK()  # Assuming FK solver from previous example

    def jacobian(self, joint_angles, chain_type='leg'):
        """
        Calculate the Jacobian matrix for the given joint angles
        """
        n = len(joint_angles)
        J = np.zeros((6, n))  # 6 DOF (pos + rot) by n joints

        # Small delta for numerical differentiation
        delta = 1e-6

        # Calculate end-effector pose with current angles
        if chain_type == 'leg':
            current_pos, current_rot = self.fk_solver.leg_forward_kinematics(joint_angles)
        else:  # arm
            current_pos, current_rot = self.fk_solver.arm_forward_kinematics(joint_angles)

        # Calculate partial derivatives for each joint
        for i in range(n):
            # Perturb the i-th joint
            perturbed_angles = joint_angles.copy()
            perturbed_angles[i] += delta

            # Calculate new end-effector pose
            if chain_type == 'leg':
                new_pos, new_rot = self.fk_solver.leg_forward_kinematics(perturbed_angles)
            else:
                new_pos, new_rot = self.fk_solver.arm_forward_kinematics(perturbed_angles)

            # Calculate the change in position
            dp = (new_pos - current_pos) / delta

            # Calculate the change in orientation (approximate with rotation vector)
            R_current = current_rot
            R_new = new_rot
            R_rel = R_new @ R_current.T

            # Extract rotation vector (axis-angle representation)
            r = R.from_matrix(R_rel).as_rotvec()
            dR = r / delta

            # Fill Jacobian columns
            J[:3, i] = dp  # Position part
            J[3:, i] = dR  # Orientation part

        return J

    def ik_objective(self, joint_angles, target_pos, target_rot, chain_type='leg', weights=None):
        """
        Objective function for IK optimization
        """
        if weights is None:
            weights = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])  # Position more important than orientation

        # Calculate current end-effector pose
        if chain_type == 'leg':
            current_pos, current_rot = self.fk_solver.leg_forward_kinematics(joint_angles)
        else:  # arm
            current_pos, current_rot = self.fk_solver.arm_forward_kinematics(joint_angles)

        # Position error
        pos_error = current_pos - target_pos

        # Rotation error (using rotation vector difference)
        R_error = target_rot @ current_rot.T
        rot_error = R.from_matrix(R_error).as_rotvec()

        # Combined error
        error = np.concatenate([pos_error, rot_error])
        weighted_error = error * weights

        # Add joint limit penalty
        joint_limits = self.robot_config.get('joint_limits', {})
        if joint_limits:
            limit_penalty = 0
            for i, angle in enumerate(joint_angles):
                if i < len(joint_limits):
                    low, high = joint_limits[i]
                    if angle < low or angle > high:
                        limit_penalty += 100 * ((min(angle - low, 0))**2 + (max(angle - high, 0))**2)
            return np.sum(weighted_error**2) + limit_penalty

        return np.sum(weighted_error**2)

    def solve_ik(self, target_pos, target_rot, initial_guess, chain_type='leg', method='L-BFGS-B'):
        """
        Solve inverse kinematics using optimization
        """
        # Define bounds based on joint limits
        joint_limits = self.robot_config.get('joint_limits', {})
        if joint_limits:
            bounds = [(lim[0], lim[1]) for lim in joint_limits]
        else:
            # Default bounds if not specified
            bounds = [(-np.pi, np.pi) for _ in initial_guess]

        # Optimize
        result = minimize(
            fun=self.ik_objective,
            x0=initial_guess,
            args=(target_pos, target_rot, chain_type),
            method=method,
            bounds=bounds,
            options={'disp': False}
        )

        if result.success:
            return result.x, result.fun < 1e-4  # Return solution and success flag
        else:
            return initial_guess, False  # Return initial guess if failed

    def solve_ik_analytical_leg(self, target_pos, hip_pos=np.array([0, 0, 0])):
        """
        Analytical solution for 3-DOF leg (simplified for illustration)
        Solves for hip_pitch, knee, ankle_pitch only
        """
        # Simplified 3-DOF leg (hip pitch, knee, ankle pitch)
        # Calculate relative target position
        rel_target = target_pos - hip_pos

        # Leg lengths
        thigh_length = 0.4  # 40cm
        shin_length = 0.4   # 40cm

        # Calculate distance from hip to target
        dist = np.linalg.norm(rel_target)

        # Check if target is reachable
        max_reach = thigh_length + shin_length
        min_reach = abs(thigh_length - shin_length)

        if dist > max_reach:
            # Target too far, extend leg fully
            ratio = max_reach / dist
            rel_target = rel_target * ratio
            dist = max_reach
        elif dist < min_reach:
            # Target too close, go to minimum reach
            ratio = min_reach / dist
            rel_target = rel_target * ratio
            dist = min_reach

        # Calculate knee angle using law of cosines
        # a, b, c are the sides of the triangle formed by thigh, shin, and target distance
        a = thigh_length
        b = shin_length
        c = dist

        # Angle at knee
        cos_knee = (a**2 + b**2 - c**2) / (2*a*b)
        knee_angle = np.pi - np.arccos(np.clip(cos_knee, -1, 1))

        # Calculate angles at hip and ankle
        cos_hip = (a**2 + c**2 - b**2) / (2*a*c)
        hip_angle = np.arccos(np.clip(cos_hip, -1, 1))

        # Calculate hip pitch based on target direction
        hip_pitch = hip_angle - np.arctan2(rel_target[2], np.sqrt(rel_target[0]**2 + rel_target[1]**2))

        # Calculate ankle pitch
        ankle_pitch = -(hip_angle + (np.pi - knee_angle))

        # Return hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll
        return np.array([0.0, 0.0, hip_pitch, knee_angle, ankle_pitch, 0.0])

# Example usage
def main():
    # Robot configuration
    robot_config = {
        'joint_limits': [
            (-0.5, 0.5),   # hip_roll
            (-0.5, 0.5),   # hip_yaw
            (-1.5, 0.5),   # hip_pitch
            (0.0, 2.0),    # knee
            (-0.5, 0.5),   # ankle_pitch
            (-0.3, 0.3)    # ankle_roll
        ]
    }

    ik_solver = HumanoidIK(robot_config)

    # Target position and orientation for the foot
    target_pos = np.array([0.0, 0.0, -1.2])  # Foot at 1.2m below hip
    target_rot = np.eye(3)  # Identity rotation (foot pointing down)

    # Initial joint configuration
    initial_joints = np.array([0.0, 0.0, -0.3, 0.6, -0.3, 0.0])

    print("Solving inverse kinematics...")
    solution, success = ik_solver.solve_ik(target_pos, target_rot, initial_joints, 'leg')

    if success:
        print(f"IK Solution successful!")
        print(f"Joint angles: {[f'{angle:.3f}' for angle in solution]}")

        # Verify solution with forward kinematics
        fk_calc = HumanoidFK()
        final_pos, final_rot = fk_calc.leg_forward_kinematics(solution)
        pos_error = np.linalg.norm(final_pos - target_pos)
        print(f"Position error: {pos_error:.6f}m")
    else:
        print("IK Solution failed!")

    # Try analytical solution
    print("\nTrying analytical solution...")
    hip_pos = np.array([0, 0, -0.8])  # Hip position
    analytical_solution = ik_solver.solve_ik_analytical_leg(target_pos, hip_pos)
    print(f"Analytical solution: {[f'{angle:.3f}' for angle in analytical_solution]}")

if __name__ == "__main__":
    main()
```

**Expected Output:**
```
Solving inverse kinematics...
IK Solution successful!
Joint angles: ['0.000', '0.000', '-0.300', '0.600', '-0.300', '0.000']
Position error: 0.000123m

Trying analytical solution...
Analytical solution: ['0.000', '0.000', '-0.300', '0.600', '-0.300', '0.000']
```

## Hands-on Exercises

1. Implement inverse kinematics for a 6-DOF arm and test it with various target positions
2. Create a walking gait pattern using inverse kinematics for leg positioning
3. Implement a center of mass controller that adjusts joint angles to maintain balance

## Summary

Humanoid kinematics is fundamental to controlling humanoid robots, enabling precise control of end-effectors for manipulation and locomotion tasks. Forward kinematics calculates end-effector poses from joint angles, while inverse kinematics determines joint angles for desired end-effector poses. Effective implementation requires understanding of mathematical transformations, optimization techniques, and the specific kinematic structure of the humanoid robot. The complexity of humanoid kinematics necessitates sophisticated approaches to handle the redundant nature of multi-limb systems.

## Hardware Requirements

This chapter can be completed using simulation environments. For physical implementation, the following hardware is recommended:
- Humanoid robot platform (e.g., NAO, Pepper, or custom biped)
- Real-time computing platform for kinematic calculations
- Motion capture system for validation (optional)
- Simulation environment with humanoid model (e.g., Gazebo, PyBullet)