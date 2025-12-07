# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- GitHub account for deployment

## Setup Instructions

### 1. Initialize Docusaurus Project

```bash
# Create a new Docusaurus project
npx create-docusaurus@latest website-name classic

# Navigate to project directory
cd website-name

# Install additional dependencies
npm install @docusaurus/module-type-aliases @docusaurus/types
npm install prism-react-renderer # For syntax highlighting
npm install @docusaurus/preset-classic # For standard preset
```

### 2. Configure Docusaurus

Update `docusaurus.config.js` with the following configuration:

```javascript
// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive 13-week curriculum for Computer Science and Engineering students',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages, this is usually '/project-name/'
  baseUrl: '/your-repo-name/',

  // GitHub pages deployment config
  organizationName: 'your-username', // Usually your GitHub org/user name
  projectName: 'your-repo-name', // Usually your repo name
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-username/your-repo-name/tree/main/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/your-username/your-repo-name',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Content',
            items: [
              {
                label: 'Week 1-2: Physical AI Foundations',
                to: '/docs/week-01/foundations-of-physical-ai',
              },
              // Add more week links as needed
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/your-repo-name',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'yaml', 'xml', 'urdf'],
      },
    }),
};

export default config;
```

### 3. Set up Content Structure

Create the weekly content directories:

```bash
mkdir -p docs/week-{01..13}
mkdir -p static/img/week-{01..13}
```

### 4. Configure Sidebar Navigation

Create `sidebars.js`:

```javascript
// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Week 1-2: Physical AI Foundations',
      items: [
        'week-01/foundations-of-physical-ai',
        'week-01/sensor-systems',
        'week-02/embodied-intelligence',
        'week-02/lidar-cameras-imus',
      ],
    },
    {
      type: 'category',
      label: 'Week 3-5: ROS 2 Fundamentals',
      items: [
        'week-03/ros2-architecture',
        'week-04/building-ros2-packages',
        'week-05/launch-files-urdf-basics',
      ],
    },
    {
      type: 'category',
      label: 'Week 6-7: Robot Simulation',
      items: [
        'week-06/gazebo-environment-setup',
        'week-07/unity-robot-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Week 8-10: NVIDIA Isaac Platform',
      items: [
        'week-08/isaac-sdk-setup',
        'week-09/ai-powered-perception',
        'week-10/reinforcement-learning-transfer',
      ],
    },
    {
      type: 'category',
      label: 'Week 11-12: Humanoid Development',
      items: [
        'week-11/humanoid-kinematics',
        'week-12/manipulation-grasping',
      ],
    },
    {
      type: 'category',
      label: 'Week 13: Conversational Robotics',
      items: [
        'week-13/conversational-robotics',
      ],
    },
  ],
};

export default sidebars;
```

### 5. Create GitHub Actions Workflow

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  test-deploy:
    if: github.event_name != 'push'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Test build
        run: npm run build

  deploy:
    if: github.event_name == 'push'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Build website
        run: npm run build

      # Popular action to deploy to GitHub Pages:
      # Docs: https://github.com/peaceiris/actions-gh-pages#%EF%B8%8F-docusaurus
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          # Build output to publish to the `gh-pages` branch:
          publish_dir: ./build
          # The following lines assign commit authorship to the official
          # GH-Actions bot for deploys to `gh-pages` branch:
          # https://github.com/actions/checkout/issues/13#issuecomment-724415212
          # If your repo uses a different deploy key, you can override these
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

### 6. Create a Sample Chapter Template

Create `docs/week-01/foundations-of-physical-ai.md` as a template:

```markdown
---
sidebar_position: 1
title: "Foundations of Physical AI and Embodied Intelligence"
---

# Foundations of Physical AI and Embodied Intelligence

## Learning Objectives

By the end of this chapter, you will be able to:
- Define Physical AI and embodied intelligence
- Explain the relationship between AI and physical systems
- Identify key challenges in Physical AI
- Describe applications of Physical AI in robotics

## Prerequisites

- Basic understanding of AI concepts
- Familiarity with programming concepts

## Theory

Physical AI represents a paradigm shift from traditional AI that operates primarily in digital spaces to AI systems that interact with and operate in the physical world. Unlike classical AI which processes abstract data, Physical AI must navigate the complexities of real-world physics, uncertainty, and embodied interaction.

The concept of embodied intelligence suggests that intelligence emerges from the interaction between an agent and its environment. This perspective challenges the traditional view of intelligence as purely computational, emphasizing instead the role of the body and environment in shaping cognitive processes.

[Continue with 500-1000 words of theory content]

## Code Example 1: Basic Physical Simulation

```python
# Purpose: Demonstrates basic physics simulation for a mobile robot
# Setup Instructions: Install numpy and matplotlib
# Run: python basic_simulation.py

import numpy as np
import matplotlib.pyplot as plt

class MobileRobot:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x  # x position
        self.y = y  # y position
        self.theta = theta  # orientation

    def move(self, v, omega, dt):
        """Move the robot with linear velocity v and angular velocity omega"""
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

# Example usage
robot = MobileRobot()
print(f"Initial position: ({robot.x}, {robot.y}, {robot.theta})")

# Move the robot forward
robot.move(v=1.0, omega=0.1, dt=0.1)
print(f"New position: ({robot.x:.2f}, {robot.y:.2f}, {robot.theta:.2f})")
```

**Expected Output:**
```
Initial position: (0, 0, 0)
New position: (0.10, 0.00, 0.01)
```

## Code Example 2: Sensor Data Processing

```python
# Purpose: Process sensor data from a simulated LIDAR
# Setup Instructions: Install numpy
# Run: python sensor_processing.py

import numpy as np

def process_lidar_data(raw_data):
    """Process raw LIDAR data to detect obstacles"""
    # Convert to numpy array for processing
    distances = np.array(raw_data)

    # Define obstacle threshold (in meters)
    obstacle_threshold = 1.0

    # Find indices where obstacles are detected
    obstacle_indices = np.where(distances < obstacle_threshold)[0]

    # Calculate obstacle positions
    angle_increment = 2 * np.pi / len(distances)
    obstacle_angles = obstacle_indices * angle_increment

    return {
        'obstacle_distances': distances[obstacle_indices],
        'obstacle_angles': obstacle_angles,
        'obstacle_count': len(obstacle_indices)
    }

# Example usage
lidar_data = [2.5, 1.8, 0.8, 1.2, 3.0, 0.5, 2.1]  # Simulated distances
obstacles = process_lidar_data(lidar_data)

print(f"Detected {obstacles['obstacle_count']} obstacles")
for i, (dist, angle) in enumerate(zip(obstacles['obstacle_distances'], obstacles['obstacle_angles'])):
    print(f"Obstacle {i+1}: distance={dist:.2f}m, angle={np.degrees(angle):.2f}°")
```

**Expected Output:**
```
Detected 2 obstacles
Obstacle 1: distance=0.80m, angle=102.86°
Obstacle 2: distance=0.50m, angle=261.80°
```

## Hands-on Exercises

1. Modify the MobileRobot class to include collision detection with boundaries
2. Extend the LIDAR processing to identify clusters of obstacles (representing larger objects)
3. Implement a simple path planning algorithm that avoids detected obstacles

## Summary

Physical AI and embodied intelligence represent a fundamental shift in how we conceptualize artificial intelligence. By grounding AI systems in physical reality, we can create more robust, adaptive, and capable systems that interact meaningfully with the world. This foundation is essential for developing advanced robotic systems that can operate effectively in real-world environments.

## Hardware Requirements

This chapter can be completed using simulation environments. For physical implementation, the following hardware is recommended:
- Mobile robot platform (e.g., TurtleBot3)
- LIDAR sensor (e.g., RPLIDAR A1)
- Computer with ROS 2 installed