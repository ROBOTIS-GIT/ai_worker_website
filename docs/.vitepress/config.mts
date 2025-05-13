import { defineConfig } from 'vitepress'

export default defineConfig({
  title: "AI Worker",
  description: "Documentation for AI Worker",
  appearance: 'dark',
  themeConfig: {
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Documentations', link: '/markdown-examples' },
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

    sidebar: [
      {
        text: 'Introduction',
        items: [
          { text: 'Markdown Examples', link: '/markdown-examples' },
          { text: 'Runtime API Examples', link: '/api-examples' }
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
