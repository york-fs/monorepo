import { globalIgnores } from 'eslint/config'
import { defineConfigWithVueTs, vueTsConfigs } from '@vue/eslint-config-typescript'
import pluginVue from 'eslint-plugin-vue'
import pluginOxlint from 'eslint-plugin-oxlint'
import skipFormatting from 'eslint-config-prettier/flat'

// To allow more languages other than `ts` in `.vue` files, uncomment the following lines:
// import { configureVueProject } from '@vue/eslint-config-typescript'
// configureVueProject({ scriptLangs: ['ts', 'tsx'] })
// More info at https://github.com/vuejs/eslint-config-typescript/#advanced-setup

export default defineConfigWithVueTs(
    {
        name: 'app/files-to-lint',
        files: ['**/*.{vue,ts,mts,tsx}'],
    },

    globalIgnores(['**/dist/**', '**/dist-ssr/**', '**/coverage/**']),

    ...pluginVue.configs['flat/essential'],
    vueTsConfigs.recommended,

    {
        name: 'app/component-names',
        files: ['**/*.vue'],
        rules: {
            // The shared primitives are deliberately named for the single noun
            // they represent — Tile, Section, Badge — rather than carrying a
            // Base/App prefix purely to satisfy this rule. The clash it guards
            // against can't happen here: every component is imported into a
            // `<script setup>` block, so the template compiler resolves the tag
            // to that binding statically, and a capitalised tag is treated as a
            // component regardless of any same-named HTML element.
            'vue/multi-word-component-names': 'off',
        },
    },

    ...pluginOxlint.buildFromOxlintConfigFile('.oxlintrc.json'),

    skipFormatting,
)
