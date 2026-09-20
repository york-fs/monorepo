<script setup lang="ts">
import type { Severity } from '@/domain/severity'

// The bordered box every readout in the app sits in — surface, border, radius,
// padding, a label, an optional severity accent down the left edge, and two
// content shapes.
//
// **Two layouts, chosen by whether `value` is set:**
//   - `value` given — a single headline reading: quiet name, big
//     severity-coloured value, optional `sub` row beneath.
//   - `value` omitted — a block: the label as a real heading, then whatever
//     the default slot holds (a readout list, a chart, the fuse grid).
withDefaults(
    defineProps<{
        title?: string
        /**
         * The single headline reading, already formatted by the caller. Its
         * presence is what selects the reading layout, and it implies the
         * accent bar — every reading in the app either carries a severity or
         * reserves room for one.
         */
        value?: string
        /**
         * Heading level for `title` in the block layout. 3 for a tile sitting
         * directly under its section's h2, 4 for one nested a level deeper, so
         * heading order doesn't jump. Ignored in the reading layout, where the
         * name is a plain span — a row of readings is a set of labelled
         * values, not fifteen headings in the document outline.
         */
        level?: 3 | 4
        /**
         * Reserve the 4px accent bar down the left edge. Separate from
         * `severity` because a tile whose severity is merely unknown (no
         * signal yet) still needs the bar reserved, or it would sit 3px
         * narrower than the coloured tiles beside it. Block tiles that never
         * carry a severity at all (the precharge readouts, the chart tiles)
         * leave this off and keep a plain 1px border.
         */
        accent?: boolean
        severity?: Severity
        /**
         * Stretch the body to fill the tile's height instead of sizing to
         * content — for charts, whose height comes purely from CSS (see
         * TimeSeries.vue's `.chart-wrap`). Block layout only.
         */
        chart?: boolean
    }>(),
    { title: undefined, value: undefined, level: 3, severity: undefined },
)
</script>

<template>
    <div
        class="tile"
        :class="{ reading: value !== undefined, accent: accent || value !== undefined, chart }"
        :data-severity="severity"
    >
        <template v-if="value !== undefined">
            <span v-if="title" class="title">{{ title }}</span>
            <span class="value">{{ value }}</span>
            <div class="sub"><slot name="sub" /></div>
        </template>
        <template v-else>
            <component :is="`h${level}`" v-if="title" class="title">{{ title }}</component>
            <slot />
        </template>
    </div>
</template>

<style scoped>
.tile {
    background: var(--surface-tile);
    border: 1px solid var(--border);
    border-radius: 0.375rem;
    padding: 0.75rem 1rem;
}

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

.tile.chart {
    display: grid;
    grid-template-rows: auto 1fr;
}

.tile.reading {
    display: grid;
    gap: var(--gap-tight);
}

.title {
    font-size: 0.8rem;
    font-weight: 550;
    text-transform: uppercase;
    color: var(--ink-muted);
}

h3.title,
h4.title {
    margin: 0 0 0.75rem;
}

.value {
    font-size: 1.375rem;
    font-variant-numeric: tabular-nums;
}
.tile[data-severity='good'] .value {
    color: var(--status-good-text);
}
.tile[data-severity='warning'] .value {
    color: var(--status-warning-text);
}
.tile[data-severity='critical'] .value {
    color: var(--status-critical-text);
}

/* Reserves the secondary row's height even when a consumer has nothing to
   put in it, so tiles sharing a row stay the same height regardless of
   whether they use the `sub` slot (see rear distribution's uptime line in
   ConnectionStatusTile). */
.sub {
    min-height: 1.1875rem;
}
</style>
