import { defineConfig } from 'vitepress'
import { tabsMarkdownPlugin } from 'vitepress-plugin-tabs'

const title = 'ROBOTIS'
const description =
  'The official e-Manual for ROBOTIS, the open source platform for Physical AI. Includes documentation, open source resources, and the latest updates for AI Sapiens, AI Worker, AI Manipulator, and ROBOTIS Hands.'
const ogUrl = 'https://ai.robotis.com/'
const ogImage = `${ogUrl}og_image.png`

/** Inline SVG — `icon: '/favicon.svg'` is treated as a Simple Icons name and triggers a broken Iconify URL. */
const robotisHomeSocialIcon = {
  svg: '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 16 16"><rect fill="#fff" width="16" height="16"/><path fill="#222" d="M13.6,14.08H9.77L5.5,9.44l1.2-2.75h3.18c0.26,0,0.49-0.09,0.67-0.28c0.18-0.18,0.28-0.41,0.28-0.67c0-0.26-0.09-0.49-0.28-0.67c-0.18-0.18-0.41-0.28-0.67-0.28H5.15v9.3H2.4V2.03h7.49c1.02,0,1.89,0.36,2.62,1.08c0.72,0.72,1.08,1.6,1.08,2.62c0,1.02-0.36,1.89-1.08,2.62c-0.72,0.72-1.59,1.08-2.61,1.08L9.28,9.43L13.6,14.08z"/></svg>'
}

export default defineConfig({
  title,
  description,
  appearance: 'dark',
  lastUpdated: true,
  sitemap: {
    hostname: 'https://ai.robotis.com'
  },
  head: [
    ['link', { rel: 'icon', href: '/favicon.svg', type: 'image/svg+xml' }],
    ['link', { rel: 'alternate icon', href: '/favicon.ico', type: 'image/png', sizes: '16x16' }],
    ['meta', { name: 'author', content: 'ROBOTIS' }],
    ['meta', { name: 'description', content: description }],
    ['meta', { name: 'keywords', content: 'AI Worker, AI Manipulator, AI, Physical AI, Embodied AI, Imitation Learning, Reinforcement Learning, Robot, Robotics, Humanoid, ROS, Simulation, Open Source, ROBOTIS,  AI Models, AI Datasets, Dual-Arm System, Robot Control, Robot Simulation, AI Robotics' }],
    ['meta', { property: 'og:title', content: title }],
    ['meta', { property: 'og:url', content: ogUrl }],
    ['meta', { property: 'og:description', content: description }],
    ['meta', { property: 'og:type', content: 'website' }],
    ['meta', { property: 'og:image', content: ogImage }],
    ['meta', { property: 'og:image:width', content: '1200' }],
    ['meta', { property: 'og:image:height', content: '630' }],
    ['meta', { property: 'og:image:alt', content: 'AI Worker Logo' }],
    ['meta', { name: 'twitter:title', content: title }],
    ['meta', { name: 'twitter:description', content: description }],
    ['meta', { name: 'twitter:card', content: 'summary_large_image' }],
    ['meta', { name: 'twitter:image', content: ogImage }],
    [
      'script',
      { async: '', src: 'https://www.googletagmanager.com/gtag/js?id=GTM-P7L347G7' }
    ],
    [
      'script',
      {},
      `window.dataLayer = window.dataLayer || [];
      function gtag(){dataLayer.push(arguments);}
      gtag('js', new Date());
      gtag('config', 'GTM-P7L347G7');`
    ]
  ],
  markdown: {
    config(md) {
      md.use(tabsMarkdownPlugin)
    }
  },
  themeConfig: {
    logo: '/favicon.svg',
    search: {
      provider: 'local'
    },
    nav: [
      { text: 'Home', link: '/' },
      { text: 'AI Sapiens', link: '/ai_sapiens/introduction_ai_sapiens' },
      { text: 'AI Worker', link: '/ai_worker/introduction_ai_worker' },
      { text: 'OMY', link: '/omy/introduction_omy' },
      { text: 'OMX', link: '/omx/introduction_omx' },
      { text: 'Hands', link: '/hands/introduction_hands' },
      { text: 'Ecosystem', link: '/community_showcase' },
      { text: 'Open Source', link: '/opensource' },
      { text: 'Contact', link: '/contact' },
    ],
    sidebar: {
      '/ai_worker/': [
        {
          text: 'AI Worker',
          items: [
            { text: 'Introduction', link: '/ai_worker/introduction_ai_worker' },
            { text: 'Video Gallery', link: '/ai_worker/videos_ai_worker' }
          ]
        },
        {
          text: 'Specifications',
          items: [
            { text: 'Hardware', link: '/ai_worker/hardware_ai_worker' },
            { text: 'Software', link: '/ai_worker/software_ai_worker' }
          ]
        },
        {
          text: 'Quick Start Guide',
          items: [
            {
              text: 'Setup Overview', link: '/ai_worker/setup_guide_ai_worker',
              items: [
                { text: 'Hardware', link: '/ai_worker/setup_guide_hardware_ai_worker' },
                { text: 'Software', link: '/ai_worker/setup_guide_software_ai_worker' },
              ]
            },
            {
              text: 'Operation Guide',
              link: '/ai_worker/operation_ai_worker',
              items: [
                { text: 'Teleoperation', link: '/ai_worker/operation_teleoperation_ai_worker' },
                { text: 'VR Teleoperation', link: '/ai_worker/operation_vr_teleoperation_ai_worker' },
                { text: 'Navigation', link: '/ai_worker/operation_navigation_ai_worker' },
              ]
            }
          ]
        },
        {
          text: 'Imitation Learning',
          items: [
            { text: 'Overview', link: '/ai_worker/imitation_learning_ai_worker' },
            {
              text: 'Dataset Preparation', link: '/ai_worker/dataset_preparation_ai_worker',
              items: [
                { text: 'Prerequisites', link: '/ai_worker/dataset_preparation_prerequisites_ai_worker' },
                { text: 'Recording', link: '/ai_worker/dataset_preparation_recording_ai_worker' },
                { text: 'Visualization', link: '/ai_worker/dataset_preparation_visualization_ai_worker' },
              ]
            },
            {
              text: 'Model Training', link: '/ai_worker/model_training_ai_worker',
            },
            {
              text: 'Model Inference', link: '/ai_worker/model_inference_ai_worker',
            },
            { text: 'Data Tools', link: '/ai_worker/data_tools_ai_worker' },
          ]
        },
        {
          text: 'Simulation',
          items: [
            { text: 'Overview', link: '/ai_worker/simulation_ai_worker' },
            { text: 'Gazebo', link: '/ai_worker/gazebo_ai_worker' },
            { text: 'Isaac Sim/Lab', link: '/ai_worker/robotis_lab_ai_worker' }
          ]
        },
        {
          text: 'Advanced Features',
          items: [
            { text: 'Overview', link: '/ai_worker/advanced_features_ai_worker' },
            { text: 'Behavior Trees', link: '/ai_worker/behavior_tree_ai_worker' },
            { text: 'Cyclo Control', link: '/ai_worker/advanced_motion_controller_ai_worker' },
          ]
        },
        {
          text: 'Resources',
          items: [
            { text: 'Open Source', link: '/opensource' },
            { text: 'Release Notes', link: '/ai_worker/release_notes_ai_worker' },
            { text: 'Technical Story', link: '/ai_worker/technical_story_ai_worker' }
          ]
        },
        {
          text: 'Support',
          items: [
            { text: 'Troubleshooting Guide', link: '/ai_worker/multiturn_clear_ai_worker' },
            { text: 'Discord Server', link: 'https://discord.gg/robotis', target: '_blank' },
            { text: 'Issues', link: '/ai_worker/issues_ai_worker' },
            { text: 'FAQ', link: '/ai_worker/faq_ai_worker' },
            { text: 'Contact Us', link: '/contact' }
          ]
        }
      ],

      '/omy/': [
        {
          text: 'OMY',
          items: [
            { text: 'Introduction', link: '/omy/introduction_omy' },
            { text: 'Video Gallery', link: '/omy/videos_omy' }
          ]
        },
        {
          text: 'Specifications',
          items: [
            {
              text: 'Hardware', 
              link: '/omy/hardware_omy', 
              collapsed: false,
              items: [
                { text: 'Control Table', link: '/omy/control_table_omy_unit' }
              ]
            },
            { text: 'Software', link: '/omy/software_omy' }
          ]
        },
        {
          text: 'Quick Start Guide',
          items: [
            { text: 'Setup Guide', link: '/omy/setup_guide_omy' },
            { text: 'Operation Guide', link: '/omy/operation_omy' }
          ]
        },
        {
          text: 'Imitation Learning',
          items: [
            { text: 'Overview', link: '/omy/imitation_learning_omy' },
            {
              text: 'Dataset Preparation', link: '/omy/dataset_preparation_omy',
              items: [
                { text: 'Prerequisites', link: '/omy/dataset_preparation_prerequisites_omy' },
                { text: 'Recording', link: '/omy/dataset_preparation_recording_omy' },
                { text: 'Visualization', link: '/omy/dataset_preparation_visualization_omy' },
              ]
            },
            {
              text: 'Model Training', link: '/omy/model_training_omy',
            },
            {
              text: 'Model Inference', link: '/omy/model_inference_omy',
            },
            { text: 'Data Tools', link: '/omy/data_tools_omy' },
          ]
        },
        {
          text: 'Simulation',
          items: [
            { text: 'Overview', link: '/omy/simulation_omy' },
            { text: 'Gazebo', link: '/omy/gazebo_omy' },
            { text: 'Isaac Sim/Lab', link: '/omy/robotis_lab_omy' }
          ]
        },
        {
          text: 'Advanced Features',
          items: [
            { text: 'Overview', link: '/omy/advanced_features_omy' },
            { text: 'Cyclo Control', link: '/omy/advanced_motion_controller_omy' }
          ]
        },
        {
          text: 'Resources',
          items: [
            { text: 'Open Source', link: '/opensource' },
            { text: 'Release Notes', link: '/omy/release_notes_omy' }
          ]
        },
        {
          text: 'Support',
          items: [
            { text: 'Troubleshooting Guide', link: '/omy/multiturn_clear_omy' },
            { text: 'Manual Packing Procedure', link: '/omy/manual_packing_procedure_omy' },
            { text: 'Discord Server', link: 'https://discord.gg/robotis', target: '_blank' },
            { text: 'Issues', link: '/omy/issues_omy' },
            { text: 'FAQ', link: '/omy/faq_omy' },
            { text: 'Contact Us', link: '/contact' }
          ]
        }
      ],

      '/omx/': [
        {
          text: 'OMX',
          items: [
            { text: 'Introduction', link: '/omx/introduction_omx' },
            { text: 'Video Gallery', link: '/omx/videos_omx' }
          ]
        },
        {
          text: 'Specifications',
          items: [
            { text: 'Hardware', link: '/omx/hardware_omx' },
            { text: 'Software', link: '/omx/software_omx' }
          ]
        },
        {
          text: 'Quick Start Guide',
          items: [
            { text: 'Assembly Guide', link: '/omx/assembly_guide_omx' },
            {
              text: 'Setup Guide',
              link: '/omx/setup_guide_omx',
              collapsed: false,
              items: [
                { text: 'ROS 2 (Physical AI Tools)', link: '/omx/setup_guide_physical_ai_tools' },
                { text: 'LeRobot', link: 'https://huggingface.co/docs/lerobot/omx', target: '_blank' }
              ]
            },
            { text: 'Operation Guide - ROS 2', link: '/omx/operation_omx' }
          ]
        },
        {
          text: 'Imitation Learning',
          items: [
            { text: 'Overview', link: '/omx/imitation_learning_omx' },
            {
              text: 'ROS 2 (Physical AI Tools)',
              collapsed: false,
              items: [
                {
                  text: 'Dataset Preparation', link: '/omx/dataset_preparation_omx',
                  items: [
                    { text: 'Prerequisites', link: '/omx/dataset_preparation_prerequisites_omx' },
                    { text: 'Recording', link: '/omx/dataset_preparation_recording_omx' },
                    { text: 'Visualization', link: '/omx/dataset_preparation_visualization_omx' },
                  ]
                },
                { text: 'Model Training', link: '/omx/model_training_omx' },
                { text: 'Model Inference', link: '/omx/model_inference_omx' },
                { text: 'Data Tools', link: '/omx/data_tools_omx' },
              ]
            },
            { text: 'LeRobot', link: 'https://huggingface.co/docs/lerobot/omx', target: '_blank' }
          ]
        },
        {
          text: 'Simulation',
          items: [
            { text: 'Overview', link: '/omx/simulation_omx' },
            { text: 'Gazebo', link: '/omx/gazebo_omx' },
          ]
        },
        {
          text: 'Advanced Features',
          items: [
            { text: 'Overview', link: '/omx/advanced_features_omx' },
            { text: 'Cyclo Control', link: '/omx/advanced_motion_controller_omx' }
          ]
        },
        {
          text: 'Resources',
          items: [
            { text: 'Open Source', link: '/opensource' },
            { text: 'Release Notes', link: '/omx/release_notes_omx' }
          ]
        },
        {
          text: 'Support',
          items: [
            { text: 'Discord Server', link: 'https://discord.gg/robotis', target: '_blank' },
            { text: 'Issues', link: '/omx/issues_omx' },
            { text: 'FAQ', link: '/omx/faq_omx' },
            { text: 'Contact Us', link: '/contact' }
          ]
        }
      ],

      '/ai_sapiens/': [
        {
          text: 'AI Sapiens',
          items: [
            { text: 'Introduction', link: '/ai_sapiens/introduction_ai_sapiens' }
          ]
        }
      ],

      '/hands/': [
        {
          text: 'Hands',
          items: [
            { text: 'Introduction', link: '/hands/introduction_hands' },
            { text: 'Video Gallery', link: '/hands/videos_hands' }
          ]
        },
        {
          text: 'Specifications',
          items: [
            {
              text: 'Hardware',
              link: '/hands/hardware_hands',
              collapsed: false,
              items: [
                { text: 'Control Table', link: '/hands/control_table_hands' }
              ]
            },
            { text: 'Software', link: '/hands/software_hands' }
          ]
        },
        {
          text: 'Quick Start Guide',
          items: [
            { text: 'Setup Guide', link: '/hands/setup_guide_hands' },
            { text: 'Operation Guide', link: '/hands/operation_hands' },
            // { text: 'VR Teleoperation Guide', link: '/hands/operation_vr_teleoperation' },
            { text: 'DXL Wizard Guide', link: '/hands/wizard_guide' }
          ]
        },
        {
          text: 'Simulation',
          items: [
            { text: 'Overview', link: '/hands/simulation_hands' },
            { text: 'Gazebo', link: '/hands/gazebo_hands' }
          ]
        },
        {
          text: 'Resources',
          items: [
            { text: 'Open Source', link: '/opensource' },
            { text: 'Release Notes', link: '/hands/release_notes_hands' }
          ]
        },
        {
          text: 'Support',
          items: [
            { text: 'Discord Server', link: 'https://discord.gg/robotis', target: '_blank' },
            { text: 'Issues', link: '/hands/issues_hands' },
            { text: 'FAQ', link: '/hands/faq_hands' },
            { text: 'Contact Us', link: '/contact' }
          ]
        }
      ],

      '/contact': [
        {
          text: 'Contact',
          items: [{ text: 'Contact Us', link: '/contact' }]
        }
      ],

      '/opensource': [
        {
          text: 'Open Source',
          items: [{ text: 'Overview', link: '/opensource' }]
        }
      ],

      '/': [
        {
          text: 'Ecosystem',
          items: [
            { text: 'Community Showcase', link: '/community_showcase' }
          ]
        }
      ]
    },
    socialLinks: [
      {
        icon: robotisHomeSocialIcon,
        link: 'https://robotis.com/',
        ariaLabel: 'ROBOTIS'
      },
      { icon: 'github', link: 'https://github.com/ROBOTIS-GIT/' },
      { icon: 'youtube', link: 'https://www.youtube.com/@ROBOTISOpenSourceTeam' },
      { icon: 'x', link: 'https://x.com/ROBOTISAmerica' },
      { icon: 'instagram', link: 'https://www.instagram.com/robotis_global/' },
      { icon: 'facebook', link: 'https://www.facebook.com/robotis.company' },
      { icon: 'linkedin', link: 'https://www.linkedin.com/company/robotis/' },
      { icon: 'discord', link: 'https://discord.gg/robotis' }
    ],
    footer: {
      message: 'AI Worker and AI Manipulator released under the Apache-2.0 license.',
      copyright: 'Copyright © 2025 ROBOTIS. All Rights Reserved.'
    }
  }
})
