<script setup lang="ts">
import { computed } from 'vue'
import segmentSidesSvg from '@/assets/bms/segment-sides.svg?raw'
import BoundSvg, { type SvgBinding } from '@/components/bms/BoundSvg.vue'
import { useSegments } from '@/composables/useSegments'
import { cellTemperatureSeverity, cellVoltageSeverity } from '@/domain/bms'
import { FITTED_CELL_COUNT, THERMISTOR_INDEX } from '@/domain/segments'
import { severityOf } from '@/domain/severity'
import { formatCelsius } from '@/utils/formatCelsius'
import { formatVolts } from '@/utils/formatVolts'

// Both faces of one segment, every cell group and thermistor where it sits.
// The drawing is the same for every segment, so its `seg0` ids are reused for
// whichever one is selected.
const props = defineProps<{ segment: number }>()

const { readings, isStale } = useSegments()
const stale = computed(() => isStale(props.segment))

const bindings = computed<SvgBinding[]>(() => {
    const segment = readings.get(props.segment)
    const cells = Array.from({ length: FITTED_CELL_COUNT }, (_, cell): SvgBinding[] => {
        const volts = segment?.cell_voltages[cell] ?? undefined
        return [
            {
                selector: `[data-cell="${cell}"]`,
                attrs: {
                    'data-severity': severityOf(volts, cellVoltageSeverity),
                    'data-degraded': segment?.degraded_cells.includes(cell) ? '' : undefined,
                },
            },
            { selector: `#seg0-cell${cell}-value`, text: formatVolts(volts, 3) },
        ]
    })
    const thermistors = Object.entries(THERMISTOR_INDEX).map(([label, index]): SvgBinding[] => {
        const celsius = segment?.temperatures[index] ?? undefined
        return [
            {
                selector: `[data-thermistor="${label}"]`,
                attrs: {
                    'data-severity': severityOf(celsius, cellTemperatureSeverity),
                },
            },
            { selector: `#seg0-therm-${label}-value`, text: formatCelsius(celsius) },
        ]
    })
    return [...cells, ...thermistors].flat()
})
</script>

<template>
    <BoundSvg class="sides" :class="{ stale }" :svg="segmentSidesSvg" :bindings="bindings" />
</template>

<style scoped>
.sides.stale {
    opacity: 0.4;
}

.sides :deep(.frame) {
    fill: var(--surface-page);
}

.sides :deep(.busbar) {
    fill: color-mix(in srgb, var(--ink-muted) 30%, var(--surface-tile));
    stroke: var(--ink-muted);
    stroke-width: 1.5;
}

.sides :deep(.cell) {
    --tint: var(--ink-muted);
    fill: color-mix(in srgb, var(--tint) 35%, var(--surface-tile));
    stroke: var(--border);
}
.sides :deep(.cell[data-severity='warning']) {
    --tint: var(--status-warning);
}
.sides :deep(.cell[data-severity='critical']) {
    --tint: var(--status-critical);
}
.sides :deep(.cell[data-degraded]) {
    stroke: var(--status-warning);
    stroke-dasharray: 4 3;
    stroke-width: 2;
}

.sides :deep(.thermistor) {
    --tint: var(--ink-muted);
    fill: color-mix(in srgb, var(--tint) 45%, var(--surface-page));
}
.sides :deep(.thermistor[data-severity='good']) {
    --tint: var(--status-good);
}
.sides :deep(.thermistor[data-severity='warning']) {
    --tint: var(--status-warning);
}
.sides :deep(.thermistor[data-severity='critical']) {
    --tint: var(--status-critical);
}

/* The drawing scales down to about half size on a laptop, so each busbar's
   tag of readings is enlarged about its own centre to stay legible. */
.sides :deep([id$='-tag']) {
    transform: scale(1.6);
    transform-box: fill-box;
    transform-origin: center;
}

.sides :deep(.tag-bg) {
    fill: var(--surface-tile);
    stroke: var(--border);
}

.sides :deep(.tag-label) {
    font-size: 14px;
    font-weight: 600;
    fill: var(--ink-secondary);
}
.sides :deep(.cell-value),
.sides :deep(.therm-value) {
    font-size: 15px;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
}
.sides :deep(.side-title) {
    font-size: 28px;
    font-weight: 700;
}
.sides :deep(.side-sub) {
    font-size: 15px;
    fill: var(--ink-muted);
}
</style>
