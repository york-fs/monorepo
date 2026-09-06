// Self-hosted (bundled by Vite) rather than a Google Fonts <link>: this
// dashboard is expected to run on a paddock network with no internet, where
// a CDN font silently falls back to the system stack with every bold weight
// synthesised. The variable build also makes the 600/650/700 weights used
// across the app real instances rather than faux bold.
import '@fontsource-variable/inter'

import './style.css'

import { createApp } from 'vue'
import App from './App.vue'
import router from './router'

const app = createApp(App)
app.use(router)
app.mount('#app')
