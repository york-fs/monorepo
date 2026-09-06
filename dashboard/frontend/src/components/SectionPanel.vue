<script setup lang="ts">
// The plain bordered-card panel shell — surface/border/radius/padding plus an
// optional `h3` title — factored out of PrechargeSection and
// PowertrainSection, which each defined an identical copy of these rules
// alongside identical `<div class="panel"><h3>…` markup.
//
// Deliberately separate from SeverityCard.vue: that one exists for the
// coloured severity `border-left` shared by tiles and checklist panels, and
// carries no padding or title of its own. These section panels never take a
// severity — they're containers for a readout list or a chart.
defineProps<{
    title?: string
    /**
     * Stretch the body to fill the panel's height instead of sizing to
     * content — for charts, whose height comes purely from CSS (see
     * TimeSeries.vue's `.chart-wrap`).
     */
    chart?: boolean
}>()
</script>

<template>
    <div class="panel" :class="{ 'chart-panel': chart }">
        <h3 v-if="title">{{ title }}</h3>
        <slot />
    </div>
</template>

<style scoped>
.panel {
    background: var(--surface-card);
    border: 1px solid var(--border);
    border-radius: 0.375rem;
    padding: 0.875rem 1rem;
}

/* Assumes the title + body pairing every current chart panel uses; the
   body needs an explicit `1fr` track to stretch, which a bare `auto` flow
   wouldn't give it. */
.chart-panel {
    display: grid;
    grid-template-rows: auto 1fr;
}
</style>
