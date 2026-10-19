import { createRouter, createWebHistory } from 'vue-router'
import HomeView from './views/HomeView.vue'
import { PAGES } from './pages'

// Every page routes to the same view, which keeps all of them mounted and
// only shows the current one — see HomeView for why.
const router = createRouter({
    history: createWebHistory(import.meta.env.BASE_URL),
    routes: [
        ...PAGES.map(({ name, path }) => ({ name, path, component: HomeView })),
        { path: '/:pathMatch(.*)*', redirect: '/' },
    ],
})

export default router
