<script setup lang="ts">
import { computed } from 'vue'
import packTopSvg from '@/assets/bms/pack-top.svg?raw'
import BoundSvg, { type SvgBinding } from '@/components/bms/BoundSvg.vue'
import { useSegments } from '@/composables/useSegments'
import { SEGMENT_LETTERS, segmentSeverity, segmentStats } from '@/domain/segments'
import { formatCelsius } from '@/utils/formatCelsius'
import { formatVolts } from '@/utils/formatVolts'

// The pack from above, each segment coloured by its worst state and showing
// its hottest thermistor and lowest cell. Click a segment to inspect it.
const selected = defineModel<number>({ required: true })

const { readings, isStale, errorFlags } = useSegments()

const bindings = computed<SvgBinding[]>(() =>
    SEGMENT_LETTERS.flatMap((_, id) => {
        const segment = readings.get(id)
        const { minCellVoltage, maxTemperature } = segmentStats(segment)
        const stale = isStale(id) ? '' : undefined
        return [
            {
                selector: `#pack-seg${id}`,
                attrs: {
                    'data-severity': segmentSeverity(errorFlags.value?.[id], segment),
                    'data-selected': id === selected.value ? '' : undefined,
                },
            },
            {
                selector: `#pack-seg${id}-maxtemp`,
                text: formatCelsius(maxTemperature),
                attrs: { 'data-stale': stale },
            },
            {
                selector: `#pack-seg${id}-mincell`,
                text: formatVolts(minCellVoltage, 3),
                attrs: { 'data-stale': stale },
            },
        ]
    }),
)

// Each segment's shape, label and readings share a group, but only the
// shape carries its id.
function pick(event: MouseEvent) {
    const group = (event.target as Element).closest('g')
    const segment = group?.querySelector('[data-segment]')?.getAttribute('data-segment')
    if (segment) selected.value = Number(segment)
}
</script>

<template>
    <BoundSvg class="pack" :svg="packTopSvg" :bindings="bindings" @click="pick" />
</template>

<style scoped>
.pack :deep([data-segment]) {
    cursor: pointer;
}

.pack :deep(.segment) {
    --tint: var(--ink-muted);
    fill: color-mix(in srgb, var(--tint) 10%, var(--surface-tile));
    stroke: color-mix(in srgb, var(--tint) 60%, var(--border));
    stroke-width: 2;
}
.pack :deep(.segment[data-severity='good']) {
    --tint: var(--status-good);
}
.pack :deep(.segment[data-severity='warning']) {
    --tint: var(--status-warning);
}
.pack :deep(.segment[data-severity='critical']) {
    --tint: var(--status-critical);
}
.pack :deep(.segment[data-selected]) {
    stroke: var(--series-1);
    stroke-width: 5;
}

.pack :deep(.seg-label) {
    font-size: 20px;
    font-weight: 700;
}
.pack :deep(.seg-key) {
    font-size: 12px;
    fill: var(--ink-muted);
}
.pack :deep(.seg-value) {
    font-size: 22px;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
}
.pack :deep(.seg-value[data-stale]) {
    opacity: 0.4;
}
</style>
