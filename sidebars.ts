import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
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
