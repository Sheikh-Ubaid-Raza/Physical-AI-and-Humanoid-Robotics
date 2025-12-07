import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive 13-week curriculum for Computer Science and Engineering students',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://sheikh-ubaid-raza.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Physical-AI-and-Humanoid-Robotics/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Sheikh-Ubaid-Raza', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-and-Humanoid-Robotics', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from

  onBrokenLinks: 'throw',
  markdown: {
  hooks: {
    onBrokenMarkdownLinks: 'warn',
  },
},

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
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Sheikh-Ubaid-Raza/Physical-AI-and-Humanoid-Robotics/edit/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Sheikh-Ubaid-Raza/Physical-AI-and-Humanoid-Robotics/edit/main/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
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
          href: 'https://github.com/Sheikh-Ubaid-Raza/Physical-AI-and-Humanoid-Robotics',
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
            {
              label: 'Week 1-2: Sensor Systems',
              to: '/docs/week-01/sensor-systems',
            },
            {
              label: 'Week 3-4: ROS 2 Architecture',
              to: '/docs/week-03/ros2-architecture',
            },
            {
              label: 'Week 5-6: Humanoid Kinematics',
              to: '/docs/week-11/humanoid-kinematics',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/Sheikh-Ubaid-Raza/Physical-AI-and-Humanoid-Robotics',
            },
            {
              label: 'Contributing Guide',
              href: 'https://github.com/Sheikh-Ubaid-Raza/Physical-AI-and-Humanoid-Robotics/blob/main/CONTRIBUTING.md',
            },
            {
              label: 'Issues',
              href: 'https://github.com/Sheikh-Ubaid-Raza/Physical-AI-and-Humanoid-Robotics/issues',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Docusaurus',
              href: 'https://docusaurus.io',
            },
            {
              label: 'React',
              href: 'https://reactjs.org',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'yaml', 'markup', 'bash', 'json', 'cpp', 'docker'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
