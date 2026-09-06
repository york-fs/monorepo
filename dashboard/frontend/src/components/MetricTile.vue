<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import MinMaxSub from '@/components/MinMaxSub.vue'
import { useMinMax } from '@/composables/useMinMax'
import type { Severity } from '@/domain/severity'
import type { TelemetryFrame } from '@/telemetry'

// The "live numeric reading + ever-min/max sub-row" tile, shared by every
// scalar readout in the app (LV rail voltage, speed, DC input voltage, both
// temperatures, pedal travel). Was five near-identical components differing
// only in which field they read, how they format it, and which severity
// function they key their colour off — all three of which are now props.
//
// Keeps the same split as AccentTile: callers own their domain vocabulary
// (the formatter and the severity mapping), this only knows the shape.
const props = defineProps<{
    name: string
    /** The live reading, already read off the frame by the caller. */
    value?: number
    /** Renders a present value; `undefined` shows an em dash instead. */
    format: (value: number) => string
    /**
     * Reads this tile's reading off each raw frame, for the ever-range.
     * Separate from `value` because the extrema track every frame that
     * arrives (see useMinMax), not just whatever the parent last rendered.
     * Omit to drop the sub-row entirely.
     */
    select?: (frame: TelemetryFrame) => number | undefined
    /** Only when the ever-max reads a different field — see useMinMax. */
    selectMax?: (frame: TelemetryFrame) => number | undefined
    /** Omit where there's no fault condition to colour off (e.g. speed). */
    severityOf?: (value: number) => Severity
}>()

const { everMin, everMax } = useMinMax(
    (frame) => props.select?.(frame),
    (frame) => (props.selectMax ?? props.select)?.(frame),
)

function display(value: number | undefined): string {
    return value === undefined ? '—' : props.format(value)
}

function severityFor(value: number | undefined): Severity | undefined {
    return value === undefined || props.severityOf === undefined
        ? undefined
        : props.severityOf(value)
}

const severity = computed(() => severityFor(props.value))
const everMinSeverity = computed(() => severityFor(everMin.value))
const everMaxSeverity = computed(() => severityFor(everMax.value))
</script>

<template>
    <AccentTile :name="name" :severity="severity">
        {{ display(value) }}
        <template v-if="select" #sub>
            <MinMaxSub
                :min-label="display(everMin)"
                :min-severity="everMinSeverity"
                :max-label="display(everMax)"
                :max-severity="everMaxSeverity"
            />
        </template>
    </AccentTile>
</template>
