import DefaultTheme from 'vitepress/theme'
import YouTube from './components/YouTube.vue'
import './custom.css'

export default {
  ...DefaultTheme,
  enhanceApp({ app }) {
    app.component('YouTube', YouTube)
  }
}
