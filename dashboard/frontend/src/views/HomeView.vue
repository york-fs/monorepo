<script setup lang="ts">
import type { Component } from 'vue'
import { useRoute } from 'vue-router'
import Badge from '@/components/Badge.vue'
import PageTabs from '@/components/PageTabs.vue'
import SwitchToggle from '@/components/SwitchToggle.vue'
import StatusStrip from '@/components/status-bar/StatusStrip.vue'
import OverviewPage from '@/views/OverviewPage.vue'
import PredrivePage from '@/views/PredrivePage.vue'
import DrivePage from '@/views/DrivePage.vue'
import BmsPage from '@/views/BmsPage.vue'
import DebugPage from '@/views/DebugPage.vue'
import { useAutoNavigation, useAutoSwitchSetting } from '@/composables/useAutoNavigation'
import { PAGES, type PageName } from '@/pages'
import { isDemoMode } from '@/demo'

// Every page stays mounted and only the current one is shown. The charts and
// ever-ranges buffer frames per component from when they mount, so unmounting
// a page on every tab switch would throw away its history.
const PAGE_CONTENT: Record<PageName, Component> = {
    overview: OverviewPage,
    predrive: PredrivePage,
    drive: DrivePage,
    bms: BmsPage,
    debug: DebugPage,
}

const route = useRoute()
const demoMode = isDemoMode()

const { autoSwitch } = useAutoSwitchSetting()
useAutoNavigation()
</script>

<template>
    <main class="home">
        <header class="header">
            <div class="title-row">
                <h1>YFS-03 Telemetry</h1>
                <Badge v-if="demoMode">Demo mode — synthetic data</Badge>
                <PageTabs class="tabs" />
                <SwitchToggle v-model="autoSwitch">Auto-switch</SwitchToggle>
            </div>
            <StatusStrip />
        </header>
        <div v-for="page in PAGES" v-show="route.name === page.name" :key="page.name" class="page">
            <component :is="PAGE_CONTENT[page.name]" />
        </div>
    </main>
</template>

<style scoped>
.home {
    display: grid;
    gap: var(--gap-sections);
    max-width: 1800px;
    margin: 0 auto;
    padding: 0 1.25rem 1.5rem;
}

/* Stays put while a long page scrolls, so the status strip is always in
   view. Padded rather than offset from the top so nothing shows above it. */
.header {
    position: sticky;
    top: 0;
    z-index: 1;
    display: grid;
    gap: var(--gap-rows);
    padding: 1.25rem 0 0.75rem;
    background: var(--surface-page);
    border-bottom: 1px solid var(--border);
}

.title-row {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: var(--gap-inline);
}

.tabs {
    margin-left: auto;
}

.page {
    display: grid;
    gap: var(--gap-sections);
}
</style>
