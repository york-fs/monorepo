import { fileURLToPath, URL } from 'node:url'

import { defineConfig } from 'vite-plus'
import vue from '@vitejs/plugin-vue'
import vueDevTools from 'vite-plugin-vue-devtools'

// https://vite.dev/config/
export default defineConfig({
    lint: {
        plugins: ['eslint', 'typescript', 'unicorn', 'oxc', 'vue'],
        env: { browser: true },
        categories: { correctness: 'error' },
        rules: {
            // Single-noun primitives are imported in script setup, so Vue resolves
            // them statically and they cannot clash with native HTML elements.
            'vue/multi-word-component-names': 'off',
        },
    },
    fmt: { semi: false, singleQuote: true, printWidth: 100, tabWidth: 4 },
    run: {
        tasks: {
            build: { command: 'vp build', dependsOn: ['type-check'] },
            'type-check': { command: 'vue-tsc --build' },
        },
    },
    plugins: [vue(), vueDevTools()],
    resolve: {
        alias: {
            '@': fileURLToPath(new URL('./src', import.meta.url)),
        },
    },
    server: {
        proxy: {
            '/api': {
                target: 'http://localhost:8000',
                changeOrigin: true,
            },
        },
    },
})
