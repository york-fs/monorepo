<script setup lang="ts">
import type { Severity } from '@/domain/severity'

// The bordered box every readout in the app sits in — surface, border, radius,
// padding, a label, an optional severity accent down the left edge, and two
// content shapes. One noun for the shape: see STATUS.md on why "tile", "panel"
// and "card" collapsed into this one.
//
// Merged from what were three components: SeverityCard (the accent bar, no
// padding, no title), SectionPanel (padding and a title, no accent) and
// AccentTile (the name + big value + sub-row arrangement, which passed its
// severity down to SeverityCard and then read it back off the rendered
// `data-severity` attribute to colour its own value text).
//
// **Two layouts, chosen by whether `value` is set:**
//   - `value` given — a single headline reading: quiet name, big
//     severity-coloured value, optional `sub` row beneath.
//   - `value` omitted — a block: the label as a real heading, then whatever
//     the default slot holds (a readout list, a chart, the fuse grid).
withDefaults(
    defineProps<{
        /** The tile's label, in the quiet `.tile-label` tier. */
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
            <span v-if="title" class="tile-label">{{ title }}</span>
            <span class="value">{{ value }}</span>
            <div class="sub"><slot name="sub" /></div>
        </template>
        <template v-else>
            <component :is="`h${level}`" v-if="title" class="tile-label">{{ title }}</component>
            <slot />
        </template>
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

.tile.reading {
    display: grid;
    gap: var(--gap-tight);
}

.value {
    font-size: 1.375rem;
    font-weight: 650;
    letter-spacing: -0.01em;
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
