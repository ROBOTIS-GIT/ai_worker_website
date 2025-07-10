import DefaultTheme from 'vitepress/theme'
import YouTube from './components/YouTube.vue'
import './custom.css'
import { enhanceAppWithTabs } from 'vitepress-plugin-tabs/client'

export default {
  ...DefaultTheme,
  enhanceApp({ app }) {
    app.component('YouTube', YouTube),
    enhanceAppWithTabs(app)
  }
}
