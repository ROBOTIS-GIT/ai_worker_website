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
  themeConfig: {
    // logo: '/logo_aiworker.svg',
    logo: '/favicon.svg',
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Documentations', link: '/introduction' },
      { text: 'Videos', link: '/videos' },
      {
        text: 'OpenSource',
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
      { text: 'Contact Us', link: '/contact' },
    ],
    search: {
      provider: 'local',
    },
    sidebar: [
      {
        text: 'Overview',
        items: [
          { text: 'Introduction', link: '/introduction' },
          { text: 'Video Gallery', link: '/videos' },
        ]
      },
      {
        text: 'Specifications',
        items: [
          { text: 'Hardware', link: '/hardware' },
          { text: 'Software', link: '/software' },
        ]
      },
      {
        text: 'Quick Start Guide',
        items: [
          { text: 'Setup Guide', link: '/setup' },
          { text: 'Teleoperation Guide', link: '/operation' }
        ]
      },
      {
        text: 'Imitation Learning',
        items: [
          { text: 'Overview', link: '/imitation_learning' },
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
        text: 'Resources',
        items: [
          { text: 'Open Source', link: '/opensource' },
          { text: 'Release Notes', link: '/release_notes'},
        ]
      },
      {
          text: 'Support',
          items: [
            { text: 'Community', link: 'https://forum.robotis.com/', target: '_blank' },
            { text: 'Issues', link: '/issues' },
            { text: 'FAQ', link: '/faq' },
            { text: 'Contact Us', link: '/contact' },
          ]
        }
    ],
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
