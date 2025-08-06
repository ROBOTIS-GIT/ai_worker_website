import DefaultTheme from 'vitepress/theme'
import YouTube from './components/YouTube.vue'
import SimpleVersionSwitcher from '../components/SimpleVersionSwitcher.vue'
import Layout from './Layout.vue'
import './custom.css'
import { enhanceAppWithTabs } from 'vitepress-plugin-tabs/client'

export default {
  ...DefaultTheme,
  Layout,
  enhanceApp({ app }) {
    app.component('YouTube', YouTube)
    app.component('SimpleVersionSwitcher', SimpleVersionSwitcher)
    enhanceAppWithTabs(app)
  }
}
