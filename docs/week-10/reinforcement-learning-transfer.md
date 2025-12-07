---
sidebar_position: 1
title: "Reinforcement Learning and Sim-to-Real Transfer"
---

# Reinforcement Learning and Sim-to-Real Transfer

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement reinforcement learning algorithms for robotics tasks
- Design simulation environments that facilitate sim-to-real transfer
- Apply domain randomization techniques to improve transferability
- Evaluate and validate sim-to-real transfer performance
- Understand the challenges and solutions in transferring RL policies from simulation to real robots
- Design robust control policies that work in both simulation and reality

## Prerequisites

- Understanding of reinforcement learning fundamentals
- Experience with robotics simulation (covered in Weeks 6-9)
- Knowledge of neural networks and deep learning
- Familiarity with physics engines and their parameters

## Theory

Reinforcement Learning (RL) has emerged as a powerful approach for learning robot control policies. The key advantage of RL is its ability to learn complex behaviors through trial and error without explicit programming of the desired behavior. However, applying RL to real robots faces the challenge of sample efficiency and safety concerns, making simulation a critical component.

### Sim-to-Real Transfer Challenge

The sim-to-real transfer problem arises because there are discrepancies between simulation and reality:
- **Dynamics Mismatch**: Differences in friction, inertia, and other physical parameters
- **Sensor Noise**: Real sensors have different noise characteristics than simulated ones
- **Actuator Delays**: Real actuators have delays and bandwidth limitations not modeled in simulation
- **Environmental Factors**: Lighting, surface textures, and other environmental conditions differ

### Domain Randomization

Domain randomization is a technique that trains policies in simulation with randomized parameters to improve robustness and transferability. By exposing the policy to a wide variety of conditions during training, it learns to adapt to variations rather than overfitting to specific simulation parameters.

### Domain Adaptation

Domain adaptation techniques aim to bridge the gap between simulation and reality by learning mappings between the two domains or by adjusting the policy to work in the real domain using limited real-world data.

## Code Example 1: RL Policy Training in Simulation

```python
# rl_training/ppo_trainer.py
# Purpose: Train a PPO policy in simulation for robot navigation
# Setup Instructions: Install stable-baselines3, gymnasium, pybullet
# Run: python ppo_trainer.py

import numpy as np
import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticPolicy
import gymnasium as gym
from gymnasium import spaces
import pybullet
import pybullet_envs
import time

class CustomCNN(BaseFeaturesExtractor):
    """
    Custom CNN for processing robot observations
    """
    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256):
        super().__init__(observation_space, features_dim)

        # Calculate output size of conv layers
        n_input_channels = observation_space.shape[0]
        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4, padding=0),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
        )

        # Compute shape by doing one forward pass
        with torch.no_grad():
            n_flatten = self.cnn(
                torch.as_tensor(observation_space.sample()[None]).float()
            ).flatten().shape[0]

        self.linear = nn.Sequential(nn.Linear(n_flatten, features_dim), nn.ReLU())

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return self.linear(self.cnn(observations).flatten(start_dim=1))


class NavigationEnv(gym.Env):
    """
    Custom environment for robot navigation
    """
    def __init__(self):
        super(NavigationEnv, self).__init__()

        # Define action and observation space
        # Actions: [linear_velocity, angular_velocity]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        # Observation: [x, y, theta, vel_x, vel_y, angular_vel, scan_data]
        scan_size = 360  # 1-degree resolution LIDAR
        obs_dim = 6 + scan_size
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        # Robot state
        self.robot_pos = np.array([0.0, 0.0])
        self.robot_theta = 0.0
        self.robot_vel = np.array([0.0, 0.0])
        self.robot_ang_vel = 0.0

        # Goal position
        self.goal_pos = np.array([5.0, 5.0])

        # Episode parameters
        self.max_steps = 500
        self.current_step = 0
        self.reset()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Reset robot to random position near origin
        self.robot_pos = np.random.uniform(-1, 1, size=2)
        self.robot_theta = np.random.uniform(-np.pi, np.pi)
        self.robot_vel = np.array([0.0, 0.0])
        self.robot_ang_vel = 0.0
        self.current_step = 0

        # Randomize goal position
        self.goal_pos = np.random.uniform(2, 8, size=2)

        observation = self._get_observation()
        return observation, {}

    def step(self, action):
        # Unpack action
        linear_vel_cmd = action[0]
        angular_vel_cmd = action[1]

        # Update robot state (simple kinematic model)
        dt = 0.1  # Time step
        self.robot_pos[0] += self.robot_vel[0] * dt
        self.robot_pos[1] += self.robot_vel[1] * dt
        self.robot_theta += self.robot_ang_vel * dt

        # Apply velocity commands with some damping
        self.robot_vel[0] = linear_vel_cmd * np.cos(self.robot_theta) * 0.9
        self.robot_vel[1] = linear_vel_cmd * np.sin(self.robot_theta) * 0.9
        self.robot_ang_vel = angular_vel_cmd * 0.9  # Damping

        # Simulate LIDAR scan (simplified)
        scan_data = self._simulate_scan()

        # Calculate reward
        reward = self._calculate_reward()

        # Check termination conditions
        terminated = self._check_termination()
        truncated = self.current_step >= self.max_steps

        self.current_step += 1

        observation = self._get_observation()

        return observation, reward, terminated, truncated, {}

    def _get_observation(self):
        # Robot state
        robot_state = np.array([
            self.robot_pos[0],
            self.robot_pos[1],
            self.robot_theta,
            self.robot_vel[0],
            self.robot_vel[1],
            self.robot_ang_vel
        ])

        # Simulated LIDAR scan
        scan_data = self._simulate_scan()

        # Concatenate all observations
        observation = np.concatenate([robot_state, scan_data])
        return observation.astype(np.float32)

    def _simulate_scan(self):
        """Simulate LIDAR scan with some obstacles"""
        scan_data = np.ones(360) * 10.0  # Max range 10m

        # Add some obstacles in random positions
        num_obstacles = np.random.randint(3, 8)
        for _ in range(num_obstacles):
            # Random obstacle position
            obs_angle = np.random.uniform(0, 2*np.pi)
            obs_dist = np.random.uniform(0.5, 4.0)

            # Calculate obstacle position relative to robot
            obs_x = self.robot_pos[0] + obs_dist * np.cos(obs_angle)
            obs_y = self.robot_pos[1] + obs_dist * np.sin(obs_angle)

            # Calculate relative position
            rel_x = obs_x - self.robot_pos[0]
            rel_y = obs_y - self.robot_pos[1]

            # Convert to polar coordinates relative to robot heading
            rel_dist = np.sqrt(rel_x**2 + rel_y**2)
            rel_angle = np.arctan2(rel_y, rel_x) - self.robot_theta

            # Normalize angle to [0, 2*pi]
            rel_angle = (rel_angle + 2*np.pi) % (2*np.pi)

            # Update scan at appropriate angle (1-degree resolution)
            angle_idx = int(np.degrees(rel_angle)) % 360
            if rel_dist < scan_data[angle_idx]:
                scan_data[angle_idx] = rel_dist

        return scan_data

    def _calculate_reward(self):
        """Calculate reward based on robot's progress toward goal"""
        # Distance to goal
        dist_to_goal = np.linalg.norm(self.robot_pos - self.goal_pos)

        # Reward based on distance reduction
        base_reward = -dist_to_goal * 0.1  # Negative reward for distance

        # Bonus for getting closer to goal
        if hasattr(self, '_prev_dist_to_goal'):
            if dist_to_goal < self._prev_dist_to_goal:
                base_reward += 0.1  # Small bonus for progress
        self._prev_dist_to_goal = dist_to_goal

        # Large reward for reaching goal
        if dist_to_goal < 0.5:  # Within 0.5m of goal
            return 10.0

        # Penalty for collisions (if LIDAR detects obstacle very close)
        min_scan = np.min(self._simulate_scan())
        if min_scan < 0.2:  # Very close to obstacle
            return -1.0

        return base_reward

    def _check_termination(self):
        """Check if episode should terminate"""
        dist_to_goal = np.linalg.norm(self.robot_pos - self.goal_pos)
        return dist_to_goal < 0.5  # Reached goal


def train_navigation_policy():
    """Train a navigation policy using PPO"""

    # Create environment
    env = NavigationEnv()

    # Create policy with custom CNN
    policy_kwargs = dict(
        features_extractor_class=CustomCNN,
        features_extractor_kwargs=dict(features_dim=256),
    )

    # Create PPO agent
    model = PPO(
        "MultiInputPolicy",  # Actually using custom CNN policy
        env,
        policy_kwargs=policy_kwargs,
        verbose=1,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
    )

    print("Starting training...")
    model.learn(total_timesteps=100000)

    print("Training completed!")

    # Save the model
    model.save("navigation_policy_ppo")

    return model

if __name__ == "__main__":
    trained_model = train_navigation_policy()
    print("Model saved as navigation_policy_ppo.zip")
```

**Expected Output:**
```
Starting training...
---------------------------------
| rollout/           |          |
|    ep_len_mean     | 123.4    |
|    ep_rew_mean     | 2.34     |
| time/              |          |
|    fps             | 204      |
|    iterations      | 1        |
|    time_elapsed    | 10       |
|    total_timesteps | 2048     |
---------------------------------
---------------------------------
| rollout/           |          |
|    ep_len_mean     | 156.7    |
|    ep_rew_mean     | 3.45     |
| time/              |          |
|    fps             | 198      |
|    iterations      | 2        |
|    time_elapsed    | 20       |
|    total_timesteps | 4096     |
---------------------------------
...
Training completed!
Model saved as navigation_policy_ppo.zip
```

## Code Example 2: Domain Randomization for Sim-to-Real Transfer

```python
# sim_to_real/domain_randomization.py
# Purpose: Implement domain randomization for sim-to-real transfer
# Setup Instructions: Install numpy, pybullet
# Run: python domain_randomization.py

import numpy as np
import random
from abc import ABC, abstractmethod

class DomainRandomizationEnv:
    """
    Environment wrapper that applies domain randomization
    """
    def __init__(self, base_env, randomization_params):
        self.base_env = base_env
        self.randomization_params = randomization_params

        # Physics parameters that will be randomized
        self.physics_params = {
            'gravity': (-15.0, -5.0),  # Gravity range
            'friction': (0.1, 1.0),    # Friction coefficient
            'mass_multiplier': (0.8, 1.2),  # Mass scaling
            'restitution': (0.0, 0.5), # Bounce coefficient
            'linear_damping': (0.0, 0.1), # Linear damping
            'angular_damping': (0.0, 0.1) # Angular damping
        }

        # Sensor parameters
        self.sensor_params = {
            'noise_std': (0.0, 0.1),  # Sensor noise standard deviation
            'delay_range': (0, 0.05), # Sensor delay in seconds
        }

        # Action parameters
        self.action_params = {
            'actuator_noise': (0.0, 0.05),  # Actuator noise
            'actuator_delay': (0.0, 0.02), # Actuator delay
        }

        # Current randomized parameters
        self.current_params = {}

        # Randomize environment on initialization
        self.randomize_environment()

    def randomize_environment(self):
        """Apply domain randomization to environment parameters"""
        # Randomize physics parameters
        for param_name, (min_val, max_val) in self.physics_params.items():
            rand_val = np.random.uniform(min_val, max_val)
            self.current_params[param_name] = rand_val

            # Apply to PyBullet or other physics engine
            if param_name == 'gravity':
                # pybullet.setGravity(0, 0, rand_val)  # Example for PyBullet
                pass  # Placeholder for actual physics engine call

        # Randomize sensor parameters
        for param_name, (min_val, max_val) in self.sensor_params.items():
            rand_val = np.random.uniform(min_val, max_val)
            self.current_params[param_name] = rand_val

        # Randomize action parameters
        for param_name, (min_val, max_val) in self.action_params.items():
            rand_val = np.random.uniform(min_val, max_val)
            self.current_params[param_name] = rand_val

    def reset(self):
        """Reset environment with new randomization"""
        self.randomize_environment()
        return self.base_env.reset()

    def step(self, action):
        """Step environment with randomized parameters"""
        # Apply action noise if specified
        if 'actuator_noise' in self.current_params:
            noise_scale = self.current_params['actuator_noise']
            noisy_action = action + np.random.normal(0, noise_scale, size=action.shape)
            # Clip to action space bounds
            noisy_action = np.clip(noisy_action,
                                  self.base_env.action_space.low,
                                  self.base_env.action_space.high)
        else:
            noisy_action = action

        # Apply actuator delay simulation
        if 'actuator_delay' in self.current_params:
            delay = self.current_params['actuator_delay']
            # In a real implementation, this would buffer actions
            pass

        # Take step in base environment
        obs, reward, terminated, truncated, info = self.base_env.step(noisy_action)

        # Apply sensor noise
        if 'noise_std' in self.current_params:
            noise_std = self.current_params['noise_std']
            obs = obs + np.random.normal(0, noise_std, size=obs.shape)

        # Add domain parameters to info for monitoring
        info['domain_params'] = self.current_params.copy()

        return obs, reward, terminated, truncated, info

class CurriculumLearning:
    """
    Implements curriculum learning to gradually increase difficulty
    """
    def __init__(self, initial_complexity=0.1, max_complexity=1.0, growth_rate=0.001):
        self.initial_complexity = initial_complexity
        self.max_complexity = max_complexity
        self.growth_rate = growth_rate
        self.current_complexity = initial_complexity
        self.performance_history = []
        self.episode_count = 0

    def update_curriculum(self, episode_performance):
        """Update complexity based on performance"""
        self.performance_history.append(episode_performance)
        self.episode_count += 1

        # Keep only last 100 episodes for performance calculation
        if len(self.performance_history) > 100:
            self.performance_history.pop(0)

        # Calculate average performance
        if len(self.performance_history) > 10:  # Need minimum episodes
            avg_performance = np.mean(self.performance_history[-10:])

            # If performing well, increase complexity
            if avg_performance > 0.7:  # Threshold for good performance
                self.current_complexity = min(
                    self.current_complexity + self.growth_rate,
                    self.max_complexity
                )
            elif avg_performance < 0.3:  # Poor performance
                self.current_complexity = max(
                    self.current_complexity - self.growth_rate,
                    self.initial_complexity
                )

    def get_randomization_bounds(self):
        """Get parameter bounds based on current complexity"""
        # Scale the randomization ranges based on curriculum
        scaled_bounds = {}

        # Example scaling for physics parameters
        base_gravity_range = (-15.0, -5.0)
        base_friction_range = (0.1, 1.0)

        # Calculate scaled ranges
        gravity_delta = (base_gravity_range[1] - base_gravity_range[0]) * self.current_complexity
        friction_delta = (base_friction_range[1] - base_friction_range[0]) * self.current_complexity

        scaled_bounds['gravity'] = (
            base_gravity_range[0],
            base_gravity_range[0] + gravity_delta
        )

        scaled_bounds['friction'] = (
            base_friction_range[0],
            base_friction_range[0] + friction_delta
        )

        return scaled_bounds

def evaluate_transfer_performance(sim_model, real_env):
    """
    Evaluate how well a policy trained in simulation transfers to real environment
    """
    print("Evaluating sim-to-real transfer...")

    total_episodes = 10
    success_count = 0
    avg_episode_reward = 0

    for episode in range(total_episodes):
        obs, _ = real_env.reset()
        episode_reward = 0
        step_count = 0
        max_steps = 500

        while step_count < max_steps:
            # Get action from trained policy
            action, _ = sim_model.predict(obs, deterministic=True)

            # Take step in real environment
            obs, reward, terminated, truncated, info = real_env.step(action)
            episode_reward += reward
            step_count += 1

            if terminated or truncated:
                break

        avg_episode_reward += episode_reward

        # Define success criteria (e.g., reaching goal)
        if info.get('success', False):
            success_count += 1

        print(f"Episode {episode + 1}: Reward = {episode_reward:.2f}, "
              f"Steps = {step_count}, Success = {'Yes' if terminated else 'No'}")

    avg_reward = avg_episode_reward / total_episodes
    success_rate = success_count / total_episodes

    print(f"\nTransfer Performance:")
    print(f"Average Reward: {avg_reward:.2f}")
    print(f"Success Rate: {success_rate:.2%}")

    return avg_reward, success_rate

# Example usage
def main():
    # This would normally connect to a real robot simulation environment
    # For demonstration, we'll create a mock environment
    class MockBaseEnv:
        def __init__(self):
            self.action_space = type('ActionSpace', (), {'low': np.array([-1, -1]), 'high': np.array([1, 1])})()

        def reset(self):
            return np.random.random(366), {}  # Mock observation

        def step(self, action):
            obs = np.random.random(366)
            reward = np.random.random()
            terminated = False
            truncated = False
            info = {}
            return obs, reward, terminated, truncated, info

    # Create base environment
    base_env = MockBaseEnv()

    # Define randomization parameters
    randomization_params = {
        'physics_variations': True,
        'sensor_noise': True,
        'actuator_uncertainty': True
    }

    # Create domain randomization wrapper
    dr_env = DomainRandomizationEnv(base_env, randomization_params)

    # Create curriculum learner
    curriculum = CurriculumLearning(initial_complexity=0.1, max_complexity=0.8)

    print("Domain randomization environment created")
    print(f"Current parameters: {dr_env.current_params}")

    # Example of curriculum learning update
    for i in range(20):
        perf = np.random.random()  # Mock performance
        curriculum.update_curriculum(perf)
        bounds = curriculum.get_randomization_bounds()
        print(f"Step {i+1}: Complexity = {curriculum.current_complexity:.3f}, "
              f"Bounds = {bounds}")

if __name__ == "__main__":
    main()
```

**Expected Output:**
```
Domain randomization environment created
Current parameters: {'gravity': -12.34, 'friction': 0.67, 'mass_multiplier': 1.05, ...}
Step 1: Complexity = 0.100, Bounds = {'gravity': (-15.0, -14.4), 'friction': (0.1, 0.16)}
Step 2: Complexity = 0.101, Bounds = {'gravity': (-15.0, -14.39), 'friction': (0.1, 0.161)}
...
Step 20: Complexity = 0.120, Bounds = {'gravity': (-15.0, -14.2), 'friction': (0.1, 0.18)}
```

## Hands-on Exercises

1. Implement a domain randomization scheme for a simulated robot arm and evaluate its effectiveness
2. Train a reinforcement learning policy in simulation with domain randomization and test its transfer to a slightly different simulation
3. Design a curriculum learning approach that gradually increases the difficulty of a manipulation task

## Summary

Reinforcement learning provides a powerful approach for learning robot behaviors, but the sim-to-real transfer challenge remains significant. Domain randomization and curriculum learning are effective techniques for improving transferability by exposing policies to a wide range of conditions during training. The key to successful sim-to-real transfer lies in carefully designing the simulation to encompass the range of real-world variations while maintaining computational efficiency.

## Hardware Requirements

This chapter requires specialized hardware and software:
- GPU for training reinforcement learning models
- Simulation environment (PyBullet, Isaac Sim, or Gazebo)
- Robot platform for real-world validation (optional for basic exercises)
- Computing platform for real-time inference (NVIDIA Jetson or equivalent)