import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  courseSidebar: [
    {
      type: 'category',
      label: 'Introduction (Weeks 1-2)',
      items: [
        'introduction/overview',
        'introduction/physical-ai-fundamentals',
        'introduction/course-structure',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 (Weeks 3-5)',
      items: [
        'module-1-ros2/week-3-fundamentals',
        'module-1-ros2/week-4-communication',
        'module-1-ros2/week-5-advanced',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Weeks 6-7)',
      items: [
        'module-2-digital-twin/week-6-gazebo',
        'module-2-digital-twin/week-7-urdf-sim',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac (Weeks 8-10)',
      items: [
        'module-3-nvidia-isaac/week-8-intro',
        'module-3-nvidia-isaac/week-9-perception',
        'module-3-nvidia-isaac/week-10-manipulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & Humanoids (Weeks 11-13)',
      items: [
        'module-4-vla-humanoids/week-11-vla',
        'module-4-vla-humanoids/week-12-humanoid-basics',
        'module-4-vla-humanoids/week-13-integration',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone/project-guide'],
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        'reference/notation-guide',
        'reference/ros2-quick-reference',
        'reference/troubleshooting',
      ],
    },
  ],

  hardwareSidebar: [
    'hardware-setup/digital-twin-workstation',
    'hardware-setup/physical-ai-edge-kit',
    'hardware-setup/cloud-native-setup',
    'hardware-setup/checklists',
  ],

  assessmentsSidebar: [
    'assessments/ros2-package',
    'assessments/gazebo-simulation',
    'assessments/isaac-perception',
    'assessments/capstone-project',
  ],
};

export default sidebars;
