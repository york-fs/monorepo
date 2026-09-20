<script setup lang="ts">
// The responsive equal-width grid every row of cards in the app uses: as many
// columns as fit at `min` or wider, each taking an equal share of the row.
//
// This was the same three-line `repeat(auto-fit, minmax(…, 1fr))` rule copied
// into StatusBar, PowertrainSection, DistributionSection and
// ActivationChecklists, differing only in the minimum track width — which is
// the one part that's genuinely per-row, since it's set by how much text that
// row's cards have to fit. So it's the prop, and the gap comes from the
// shared rhythm rather than being restated at each call site.
//
// Not a general-purpose layout primitive: `auto-fit` means callers get no say
// in the column count, which is the point. A fixed two-column split that
// collapses at a breakpoint (PrechargeSection's body, the chart pairs) is a
// different shape and stays in the component that needs it.
withDefaults(defineProps<{ min?: string }>(), { min: '10.5rem' })
</script>

<template>
    <div class="auto-grid" :style="{ '--auto-grid-min': min }">
        <slot />
    </div>
</template>

<style scoped>
.auto-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(var(--auto-grid-min), 1fr));
    gap: var(--gap-tiles);
}
</style>
