import DefaultTheme from 'vitepress/theme'
import { h } from 'vue'
import YouTube from './components/YouTube.vue'
import AISapiensIntroHero from './components/AISapiensIntroHero.vue'
import CycloBrainArchitecture from './components/CycloBrainArchitecture.vue'
import CycloDataArchitecture from './components/CycloDataArchitecture.vue'
import DocsMigrationNotice from './components/DocsMigrationNotice.vue'
import './custom.css'
import { enhanceAppWithTabs } from 'vitepress-plugin-tabs/client'

export default {
  ...DefaultTheme,
  Layout() {
    return h(DefaultTheme.Layout, null, {
      'layout-bottom': () => h(DocsMigrationNotice)
    })
  },
  enhanceApp({ app }) {
    app.component('YouTube', YouTube)
    app.component('AISapiensIntroHero', AISapiensIntroHero)
    app.component('CycloBrainArchitecture', CycloBrainArchitecture)
    app.component('CycloDataArchitecture', CycloDataArchitecture)
    enhanceAppWithTabs(app)
  }
}
