<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import { useLvVoltageRange } from '@/composables/useLvVoltageRange'
import { lvVoltageSeverity } from '@/domain/lvVoltage'

const props = defineProps<{
    minVoltage?: number
}>()

const { everMin, everMax } = useLvVoltageRange()

const severity = computed(() =>
    props.minVoltage === undefined ? undefined : lvVoltageSeverity(props.minVoltage),
)
const everMinSeverity = computed(() =>
    everMin.value === undefined ? undefined : lvVoltageSeverity(everMin.value),
)
const everMaxSeverity = computed(() =>
    everMax.value === undefined ? undefined : lvVoltageSeverity(everMax.value),
)

function fmt(volts: number | undefined) {
    return volts === undefined ? '—' : `${volts.toFixed(2)} V`
}
</script>

<template>
    <AccentTile name="LV system" :severity="severity">
        {{ fmt(minVoltage) }}
        <template #sub>
            <span class="arrow-stat" :data-severity="everMinSeverity">
                <span class="arrow">↓</span>{{ fmt(everMin) }}
            </span>
            <span class="arrow-stat" :data-severity="everMaxSeverity">
                <span class="arrow">↑</span>{{ fmt(everMax) }}
            </span>
        </template>
    </AccentTile>
</template>

<style scoped>
:deep(.tile) {
    min-width: 12rem;
}

:deep(.sub) {
    display: flex;
    gap: 0.75rem;
}

.arrow-stat {
    display: inline-flex;
    align-items: center;
    gap: 0.1875rem;
    font-size: 0.6875rem;
    font-weight: 600;
    color: var(--ink-muted);
    font-variant-numeric: tabular-nums;
}
.arrow-stat[data-severity='good'] {
    color: var(--status-good-text);
}
.arrow-stat[data-severity='warning'] {
    color: var(--status-warning-text);
}
.arrow-stat[data-severity='critical'] {
    color: var(--status-critical-text);
}

.arrow {
    font-size: 0.75rem;
    line-height: 1;
}
</style>
