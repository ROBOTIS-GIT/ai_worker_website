import { defineConfig } from 'vitepress'
import { tabsMarkdownPlugin } from 'vitepress-plugin-tabs'

const title = 'ROBOTIS'
const description = 'Website for AI Worker and AI Manipulator'
const ogUrl = 'https://ai.robotis.com/'
const ogImage = `${ogUrl}og_image.png`

export default defineConfig({
  title,
  description,
  appearance: 'dark',
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
    nav: [
      { text: 'Home', link: '/' },
      { text: 'AI Worker', link: '/ai_worker/introduction_ai_worker' },
      { text: 'OMY', link: '/omy/introduction_omy' },
      { text: 'OMX', link: '/omx/introduction_omx' },
      { text: 'Dynamixel Ecosystem', link: '/community_showcase' },
      {
        text: 'OpenSource',
        items: [
          { text: 'AI Worker ROS 2 Packages', link: 'https://github.com/ROBOTIS-GIT/ai_worker', target: '_blank' },
          { text: 'AI Manipulator ROS 2 Packages', link: 'https://github.com/ROBOTIS-GIT/open_manipulator', target: '_blank' },
          { text: 'Physical AI Tools', link: 'https://github.com/ROBOTIS-GIT/physical_ai_tools', target: '_blank' },
          { text: 'Isaac Sim Models', link: 'https://github.com/ROBOTIS-GIT/robotis_lab/tree/main/source/robotis_lab/data/robots', target: '_blank' },
          { text: 'MuJoCo Models', link: 'https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie', target: '_blank' },
          { text: 'AI Worker URDF Models', link: 'https://github.com/ROBOTIS-GIT/ai_worker/tree/main/ffw_description/urdf', target: '_blank' },
          { text: 'AI Manipulator URDF Models', link: 'https://github.com/ROBOTIS-GIT/open_manipulator/tree/main/open_manipulator_description/urdf', target: '_blank' },
          { text: 'AI Models & Datasets (Hugging Face)', link: 'https://huggingface.co/ROBOTIS', target: '_blank' },
          { text: 'Docker Images', link: 'https://hub.docker.com/r/robotis/ros/tags', target: '_blank' },
          { text: 'ROBOTIS Homepage', link: 'https://en.robotis.com/', target: '_blank' },
          { text: 'Community', link: 'https://forum.robotis.com/', target: '_blank' },
          { text: 'Videos', link: 'https://www.youtube.com/@ROBOTISOpenSourceTeam', target: '_blank' },
        ]
      },
      { text: 'Contact', link: '/ai_worker/contact_ai_worker' },
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
            { text: 'Setup Guide', link: '/ai_worker/setup_guide_ai_worker' },
            { text: 'Operation Guide', link: '/ai_worker/operation_ai_worker' }
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
                { text: 'Editing', link: '/ai_worker/dataset_preparation_editing_ai_worker' }
              ]
            },
            {
              text: 'Model Training', link: '/ai_worker/model_training_ai_worker',
            },
            {
              text: 'Model Inference', link:'/ai_worker/model_inference_ai_worker',
            }
          ]
        },
        {
          text: 'Simulation',
          items: [
            { text: 'Overview', link: '/ai_worker/simulation_ai_worker'},
            { text: 'Gazebo', link: '/ai_worker/gazebo_ai_worker' },
            { text: 'Isaac Sim/Lab', link: '/ai_worker/robotis_lab_ai_worker' }
          ]
        },
        {
          text: 'Resources',
          items: [
            { text: 'Open Source', link: '/ai_worker/opensource_ai_worker' },
            { text: 'Release Notes', link: '/ai_worker/release_notes_ai_worker' }
          ]
        },
        {
          text: 'Support',
          items: [
            { text: 'Community Forum', link: 'https://forum.robotis.com/', target: '_blank' },
            { text: 'Issues', link: '/ai_worker/issues_ai_worker' },
            { text: 'FAQ', link: '/ai_worker/faq_ai_worker' },
            { text: 'Contact Us', link: '/ai_worker/contact_ai_worker' }
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
            { text: 'Hardware', link: '/omy/hardware_omy' },
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
                { text: 'Editing', link: '/omy/dataset_preparation_editing_omy' }
              ]
            },
            {
              text: 'Model Training', link: '/omy/model_training_omy',
            },
            {
              text: 'Model Inference', link:'/omy/model_inference_omy',
            }
          ]
        },
        {
          text: 'Simulation',
          items: [
            { text: 'Overview', link: '/omy/simulation_omy'},
            { text: 'Gazebo', link: '/omy/gazebo_omy' },
            { text: 'Isaac Sim/Lab', link: '/omy/robotis_lab_omy' }
          ]
        },
        {
          text: 'Resources',
          items: [
            { text: 'Open Source', link: '/omy/opensource_omy' },
            { text: 'Release Notes', link: '/omy/release_notes_omy' }
          ]
        },
        {
          text: 'Support',
          items: [
            { text: 'Community Forum', link: 'https://forum.robotis.com/', target: '_blank' },
            { text: 'Issues', link: '/omy/issues_omy' },
            { text: 'FAQ', link: '/omy/faq_omy' },
            { text: 'Contact Us', link: '/omy/contact_omy' }
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
        // {
        //   text: 'Specifications',
        //   items: [
        //     { text: 'Hardware', link: '/omx/hardware_omx' },
        //     { text: 'Software', link: '/omx/software_omx' }
        //   ]
        // },
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
                { text: 'LeRobot', link: '/omx/setup_guide_lerobot' }
              ]
            }
            // { text: 'Operation Guide', link: '/omx/operation_omx' }
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
                    { text: 'Editing', link: '/omx/dataset_preparation_editing_omx' }
                  ]
                },
                { text: 'Model Training', link: '/omx/model_training_omx' },
                { text: 'Model Inference', link: '/omx/model_inference_omx' }
              ]
            },
            { text: 'LeRobot', link: '/omx/lerobot_imitation_learning_omx' }
          ]
        },
        // {
        //   text: 'Simulation',
        //   items: [
        //     { text: 'Overview', link: '/omx/simulation_omx'},
        //     { text: 'Gazebo', link: '/omx/gazebo_omx' },
        //   ]
        // },
        {
          text: 'Resources',
          items: [
            { text: 'Open Source', link: '/omx/opensource_omx' },
            { text: 'Release Notes', link: '/omx/release_notes_omx' }
          ]
        },
        {
          text: 'Support',
          items: [
            { text: 'Community Forum', link: 'https://forum.robotis.com/', target: '_blank' },
            { text: 'Issues', link: '/omx/issues_omx' },
            { text: 'FAQ', link: '/omx/faq_omx' },
            { text: 'Contact Us', link: '/omx/contact_omx' }
          ]
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
      { icon: '/favicon.svg', link: 'https://en.robotis.com/' },
      { icon: 'github', link: 'https://github.com/ROBOTIS-GIT/ai_worker' },
      { icon: 'youtube', link: 'https://www.youtube.com/@ROBOTISOpenSourceTeam' },
      { icon: 'x', link: 'https://x.com/ROBOTISAmerica' },
      { icon: 'instagram', link: 'https://www.instagram.com/robotis_global/' },
      { icon: 'facebook', link: 'https://www.facebook.com/robotis.company' },
      { icon: 'linkedin', link: 'https://www.linkedin.com/company/robotis/' }
    ],
    footer: {
      message: 'AI Worker and AI Manipulator released under the Apache-2.0 license.',
      copyright: 'Copyright © 2025 ROBOTIS. All Rights Reserved.'
    }
  }
})
