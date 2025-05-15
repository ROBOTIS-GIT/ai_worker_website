import { defineConfig } from 'vitepress'

const title = 'AI Worker'
const description = 'Documentations for AI Worker'
const ogUrl = 'https://ai.robotis.com/'
const ogImage = `${ogUrl}og.png`

export default defineConfig({
  title: title,
  description: description,
  appearance: 'dark',
  head: [
    ['link', { rel: 'icon', href: '/favicon.svg', type: 'image/svg+xml' }],
    ['link', { rel: 'alternate icon', href: '/favicon.ico', type: 'image/png', sizes: '16x16' }],
    ['meta', { name: 'author', content: 'ROBOTIS' }],
    ['meta', { name: 'og:title', content: title }],
    ['meta', { name: 'og:description', content: description }],
    ['meta', { property: 'og:type', content: 'website' }],
    ['meta', { property: 'og:image', content: ogImage }],
  ],
  themeConfig: {
    // logo: '/logo_aiworker.png',
    logo: '/favicon.svg',
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Documentations', link: '/introduction' },
      {
        text: 'Resources',
        items: [
          { text: 'AI Worker Packages', link: 'https://github.com/ROBOTIS-GIT/ai_worker', target: '_blank' },
          { text: 'MuJoCo Model Files', link: 'https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie', target: '_blank' },
          { text: 'Physical AI Tools', link: 'https://github.com/ROBOTIS-GIT/physical_ai_tools', target: '_blank' },
          { text: 'AI Models & Datasets', link: 'https://huggingface.co/ROBOTIS', target: '_blank' },
          { text: 'Docker Files', link: 'https://hub.docker.com/r/robotis/ros/tags', target: '_blank' },
          { text: 'ROBOTIS', link: 'https://en.robotis.com/', target: '_blank' },
          { text: 'Community', link: 'https://forum.robotis.com/', target: '_blank' },
          { text: 'Manual', link: 'https://ai.robotis.com', target: '_blank' },
          { text: 'Videos', link: 'https://www.youtube.com/@ROBOTISOpenSourceTeam', target: '_blank' },
        ]
      },
    ],
    search: {
      provider: 'local',
    },
    sidebar: [
      {
        text: 'Overview',
        items: [
          { text: 'Introduction', link: '/introduction' },
        ]
      },
      {
        text: 'Specifications',
        items: [
          { text: 'Hardware Specification', link: '/specification' },
          { text: 'Software Specification', link: '/specification' }
        ]
      },
      {
        text: 'Quick Start Guide',
        items: [
          { text: 'Installation', link: '/markdown_examples' },
        ]
      },
      {
        text: 'Imitation Learning',
        items: [
          { text: 'Dataset Preparation', link: '/dataset_preparation' },
          { text: 'Model Workflow', link: '/model_workflow' },
        ]
      },
      {
        text: 'Simulation',
        items: [
          { text: 'Simulation', link: '/simulation' },
        ]
      },
      {
        text: 'OpenSource Projects',
        items: [
          { text: 'Resources', link: '/resources' },
        ]
      },
      {
          text: 'Troubleshooting',
          items: [
            { text: 'Issues', link: '/markdown_examples' },
            { text: 'FAQ', link: '/markdown_examples' }
          ]
        }
    ],
    socialLinks: [
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
