<script setup lang="ts">
import { RouterLink, useRoute } from 'vue-router'
import { usePageStatus } from '@/composables/usePageStatus'
import { PAGES } from '@/pages'

// Carry the query across so `?demo` survives switching pages.
const route = useRoute()

// Each tab is tinted by its page's state, so a fault on another page shows
// without having to go and look.
const pageStatus = usePageStatus()
</script>

<template>
    <nav class="tabs" aria-label="Pages">
        <RouterLink
            v-for="page in PAGES"
            :key="page.name"
            :to="{ name: page.name, query: route.query }"
            class="tab"
            :data-severity="pageStatus[page.name]"
        >
            {{ page.label }}
        </RouterLink>
    </nav>
</template>

<style scoped>
.tabs {
    display: flex;
    flex-wrap: wrap;
    gap: var(--gap-tight);
}

.tab {
    --tint: transparent;
    padding: 0.375rem 0.75rem;
    font-size: 0.875rem;
    font-weight: 550;
    color: var(--ink-secondary);
    text-decoration: none;
    background: color-mix(in srgb, var(--tint) 18%, transparent);
    border: 1px solid transparent;
    border-radius: var(--radius-badge);
}
.tab[data-severity='good'] {
    --tint: var(--status-good);
}
.tab[data-severity='warning'] {
    --tint: var(--status-warning);
}
.tab[data-severity='critical'] {
    --tint: var(--status-critical);
}

.tab:hover {
    color: var(--ink-primary);
    border-color: var(--border);
}

.tab[aria-current='page'] {
    color: var(--ink-primary);
    background: color-mix(in srgb, var(--tint) 18%, var(--surface-tile));
    border-color: var(--border);
    box-shadow: inset 0 -2px 0 var(--series-1);
}
</style>
