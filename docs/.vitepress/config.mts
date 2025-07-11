import { defineConfig } from 'vitepress'
import { tabsMarkdownPlugin } from 'vitepress-plugin-tabs'
import fs from 'fs'
import path from 'path'
import { fileURLToPath } from 'url'

const title = 'AI Worker'
const description = 'Documentations for AI Worker'
const ogUrl = 'https://ai.robotis.com/'
const ogImage = `${ogUrl}og_image.png`

const products = ['ai_worker', 'omy']
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const commonDir = path.resolve(__dirname, '../common')
const commonFiles = fs.readdirSync(commonDir).filter(file => file.endsWith('.md'))
const dynamicRewrites = products.reduce((acc, product) => {
  commonFiles.forEach(file => {
    const key = `${product}/${file}`
    const value = `common/${file}`
    acc[key] = value
  })
  return acc
}, {} as Record<string, string>)

const commonItems = (prefix: string) => [
  {
    text: 'Imitation Learning',
    items: [
      { text: 'Overview', link: `/${prefix}/imitation_learning` },
      { text: 'Before You Begin', link: `/${prefix}/imitation_learning_before_you_begin` },
      {
        text: 'Dataset Preparation', link: `/${prefix}/dataset_preparation`,
        items: [
          { text: 'Web UI', link: `/${prefix}/dataset_preparation_with_web_ui` },
          { text: 'LeRobot CLI', link: `/${prefix}/dataset_preparation_with_lerobot_cli` },
        ],
      },
      { text: 'Model Training', link: `/${prefix}/model_training`,
        items: [
          { text: 'LeRobot CLI', link: `/${prefix}/model_training_with_lerobot_cli` },
        ],
        },
      { text: 'Model Inference', link: `/${prefix}/model_inference`,
        items: [
          { text: 'Web UI', link: `/${prefix}/model_inference_with_web_ui` },
          { text: 'LeRobot CLI', link: `/${prefix}/model_inference_with_lerobot_cli` },
        ],
      },
    ]
  },
  {
    text: 'Simulation',
    items: [
      { text: 'AI Worker Simulation', link: `/${prefix}/simulation_ai_worker`,
        items: [
          { text: 'IsaacLab', link: `/${prefix}/robotis_lab` },
        ]
      },
      { text: 'OMY Simulation', link: `/${prefix}/simulation_omy` },
    ]
  },
  {
    text: 'Resources',
    items: [
      { text: 'Open Source', link: `/${prefix}/opensource` },
      { text: 'Release Notes', link: `/${prefix}/release_notes`},
    ]
  },
  {
    text: 'Ecosystem',
    items: [
      { text: 'Community Showcase', link: `/${prefix}/community_showcase` },
    ]
  },
  {
    text: 'Support',
    items: [
      { text: 'Community', link: 'https://forum.robotis.com/', target: '_blank' },
      { text: 'Issues', link: `/${prefix}/issues` },
      { text: 'FAQ', link: `/${prefix}/faq` },
      { text: 'Contact Us', link: `/${prefix}/contact` },
    ]
  }
]

export default defineConfig({
  title: title,
  description: description,
  appearance: 'dark',
  head: [
    ['link', { rel: 'icon', href: '/favicon.svg', type: 'image/svg+xml' }],
    ['link', { rel: 'alternate icon', href: '/favicon.ico', type: 'image/png', sizes: '16x16' }],
    ['meta', { name: 'author', content: 'ROBOTIS' }],
    ['meta', { name: 'description', content: description }],
    ['meta', { name: 'keywords', content: 'AI Worker, AI, Robotics, Humanoid, ROS, Imitation Learning, Reinforcement Learning, Simulation, Open Source, ROBOTIS, Physical AI, AI Models, AI Datasets, Dual-Arm System, Robot Control, Robot Simulation, AI Robotics' }],
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
  rewrites: dynamicRewrites,
  themeConfig: {
    // logo: '/logo_aiworker.svg',
    logo: '/favicon.svg',
    nav: [
      { text: 'Home', link: '/' },
      { text: 'AI Worker', link: '/ai_worker/introduction' },
      { text: 'OMY', link: '/omy/introduction' },
      // { text: 'Documentations', link: '/introduction' },
      // { text: 'Videos', link: '/videos' },
      // {
      //   text: 'OpenSource',
      //   items: [
      //     { text: 'AI Worker Packages', link: 'https://github.com/ROBOTIS-GIT/ai_worker', target: '_blank' },
      //     { text: 'MuJoCo Model Files', link: 'https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie', target: '_blank' },
      //     { text: 'Physical AI Tools', link: 'https://github.com/ROBOTIS-GIT/physical_ai_tools', target: '_blank' },
      //     { text: 'AI Models & Datasets', link: 'https://huggingface.co/ROBOTIS', target: '_blank' },
      //     { text: 'Docker Files', link: 'https://hub.docker.com/r/robotis/ros/tags', target: '_blank' },
      //     { text: 'ROBOTIS', link: 'https://en.robotis.com/', target: '_blank' },
      //     { text: 'Community', link: 'https://forum.robotis.com/', target: '_blank' },
      //     { text: 'Manual', link: 'https://ai.robotis.com', target: '_blank' },
      //     { text: 'Videos', link: 'https://www.youtube.com/@ROBOTISOpenSourceTeam', target: '_blank' },
      //   ]
      // },
      // { text: 'Contact Us', link: '/contact' },
    ],
    search: {
      provider: 'local',
    },
    sidebar: {
      '/ai_worker/': [
      {
        text: 'Overview',
        items: [
          { text: 'Introduction', link: '/ai_worker/introduction' ,
            items: [
              { text: 'AI Worker', link: '/ai_worker/introduction_ai_worker' },
            ]
          },
          { text: 'Video Gallery', link: '/ai_worker/videos' },
        ]
      },
      {
        text: 'Specifications',
        items: [
          { text: 'Hardware', link: '/ai_worker/hardware',
            items: [
              { text: 'AI Worker', link: '/ai_worker/hardware_ai_worker' },
            ]
          },
          { text: 'Software', link: '/ai_worker/software',
            items: [
              { text: 'AI Worker', link: '/ai_worker/software_ai_worker' },
            ]
          },
        ]
      },
      {
        text: 'Quick Start Guide',
        items: [
          {
            text: 'Setup Guide',
            link: '/ai_worker/setup_guide_ai_worker',
            items: [
              { text: 'AI Worker', link: '/ai_worker/ai_worker/setup_guide_ai_worker' },
            ]
          },
          {
            text: 'Operation Guide',
            link: '/ai_worker/operation_ai_worker',
            items: [
              { text: 'AI Worker', link: '/ai_worker/operation_ai_worker' },
            ]
          }
        ]
      },
      ...commonItems('ai_worker')
      ],

      '/omy/': [
      {
        text: 'Overview',
        items: [
          { text: 'Introduction', link: '/omy/introduction' ,
            items: [
              { text: 'OMY', link: '/omy/introduction_omy' },
            ]
          },
          { text: 'Video Gallery', link: '/omy/videos' },
        ]
      },
      {
        text: 'Specifications',
        items: [
          { text: 'Hardware', link: '/omy/hardware',
            items: [
              { text: 'OMY', link: '/omy/hardware_omy' },
            ]
          },
          { text: 'Software', link: '/omy/software',
            items: [
              { text: 'OMY', link: '/omy/software_omy' },
            ]
          },
        ]
      },
      {
        text: 'Quick Start Guide',
        items: [
          {
            text: 'Setup Guide', link: '/omy/setup_guide_omy',
            items: [
              { text: 'OMY', link: '/omy/setup_guide_omy' }
            ]
          },
          {
            text: 'Operation Guide', link: '/omy/operation_omy',
            items: [
              { text: 'OMY', link: '/omy/operation_omy' }
            ]
          }
        ]
      },
      ...commonItems('omy')
      ],
    },
    socialLinks: [
      { icon: '/favicon.svg', link: 'https://en.robotis.com/' },
      { icon: 'github', link: 'https://github.com/ROBOTIS-GIT/ai_worker' },
      { icon: 'youtube', link: 'https://www.youtube.com/@ROBOTISOpenSourceTeam' },
      { icon: 'x', link: 'https://x.com/ROBOTISAmerica' },
      { icon: 'instagram', link: 'https://www.instagram.com/robotis_global/' },
      { icon: 'facebook', link: 'https://www.facebook.com/robotis.company' },
      { icon: 'linkedin', link: 'https://www.linkedin.com/company/robotis/' },
    ],
    footer: {
      message: 'AI Worker released under the Apache-2.0 license.',
      copyright: 'Copyright © 2025 ROBOTIS. All Rights Reserved.',
    },
  }
})
