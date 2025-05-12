import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: "AI Worker",
  description: "Documentation for AI Worker",
  appearance: 'dark',
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Docs', link: '/markdown-examples' },
      { text: 'Examples', link: '/markdown-examples' },
      { text: 'API', link: '/api-examples' },
      { text: 'Tutorials', link: '/tutorials' },
      { text: 'Contributing', link: '/contributing' },
      { text: 'Changelog', link: '/changelog' },
      { text: 'Contact Us', link: '/contact' }
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
      { icon: 'github', link: 'https://github.com/ROBOTIS-GIT/ai_worker_website' },
      { icon: 'youtube', link: 'https://www.youtube.com/@ROBOTISOpenSourceTeam' },
      { icon: 'x', link: 'https://x.com/ROBOTISAmerica' },
      { icon: 'linkedin', link: 'https://www.linkedin.com/company/robotis/' },
    ]
  }
})
