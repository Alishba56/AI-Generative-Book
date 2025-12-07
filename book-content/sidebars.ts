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
  mainSidebar: [
    {
      type: 'category',
      label: 'Part 1 — Physical AI Foundations',
      items: ['chapter1'],
      link: {
        type: 'doc',
        id: 'chapter1',
      },
    },
    {
      type: 'category',
      label: 'Part 2 — ROS 2: Robotic Nervous System',
      items: ['chapter2'],
      link: {
        type: 'doc',
        id: 'chapter2',
      },
    },
    {
      type: 'category',
      label: 'Part 3 — Simulation Systems',
      items: ['chapter3'],
      link: {
        type: 'doc',
        id: 'chapter3',
      },
    },
    {
      type: 'category',
      label: 'Part 4 — NVIDIA Isaac',
      items: ['chapter4'],
      link: {
        type: 'doc',
        id: 'chapter4',
      },
    },
    {
      type: 'category',
      label: 'Part 5 — Vision-Language-Action',
      items: ['chapter5'],
      link: {
        type: 'doc',
        id: 'chapter5',
      },
    },
    {
      type: 'category',
      label: 'Part 6 — Capstone',
      items: ['chapter6'],
      link: {
        type: 'doc',
        id: 'chapter6',
      },
    },
  ],
};

export default sidebars;
