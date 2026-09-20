<script setup lang="ts">
import type { Severity } from '@/domain/severity'

// The bordered box every readout in the app sits in — surface, border, radius,
// padding, an optional label and an optional severity accent down the left
// edge. One noun for the shape: see STATUS.md on why "tile", "panel" and
// "card" collapsed into this one.
//
// Merged from what were two components — SeverityCard and SectionPanel —
// that had drifted apart by accident rather than by design: one owned the
// accent bar and carried no padding, the other owned padding and a title and
// no accent, and both declared their own identical copy of the
// surface/border/radius rules.
withDefaults(
    defineProps<{
        /** The tile's own label, in the quiet `.tile-label` tier. */
        title?: string
        /**
         * Heading level for `title`. 3 for a tile sitting directly under its
         * section's h2; 4 for one nested under a SubSection's own h3, so
         * heading order doesn't jump a level. Purely structural — both get
         * identical styling.
         */
        level?: 3 | 4
        /**
         * Reserve the 4px accent bar down the left edge. Separate from
         * `severity` because a tile whose severity is merely unknown (no
         * signal yet) still needs the bar reserved, or it would sit 3px
         * narrower than the coloured tiles beside it. Tiles that never carry
         * a severity at all (the precharge readouts, the chart tiles) leave
         * this off and keep a plain 1px border.
         */
        accent?: boolean
        severity?: Severity
        /**
         * Stretch the body to fill the tile's height instead of sizing to
         * content — for charts, whose height comes purely from CSS (see
         * TimeSeries.vue's `.chart-wrap`).
         */
        chart?: boolean
    }>(),
    { title: undefined, level: 3, severity: undefined },
)
</script>

<template>
    <div class="tile" :class="{ accent, chart }" :data-severity="severity">
        <component :is="`h${level}`" v-if="title" class="tile-label">{{ title }}</component>
        <slot />
    </div>
</template>

<style scoped>
.tile {
    background: var(--surface-tile);
    border: 1px solid var(--border);
    border-radius: var(--radius-tile);
    padding: var(--tile-padding);
}

/* Left in --border when no severity is set, so a tile with no signal yet
   stays aligned with its coloured neighbours. */
.tile.accent {
    border-left-width: 0.25rem;
}
.tile.accent[data-severity='good'] {
    border-left-color: var(--status-good);
}
.tile.accent[data-severity='warning'] {
    border-left-color: var(--status-warning);
}
.tile.accent[data-severity='critical'] {
    border-left-color: var(--status-critical);
}

/* Assumes the title + body pairing every current chart tile uses; the body
   needs an explicit `1fr` track to stretch, which a bare `auto` flow
   wouldn't give it. */
.tile.chart {
    display: grid;
    grid-template-rows: auto 1fr;
}
</style>
