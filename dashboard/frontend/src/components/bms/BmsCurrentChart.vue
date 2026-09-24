<script setup lang="ts">
import { computed } from 'vue'
import TimeSeries from '@/components/TimeSeries.vue'
import { useTelemetryHistory } from '@/composables/useTelemetryHistory'

// Both sensors measure the same DC bus current — one on each pole — so the
// two lines should sit on top of each other. Divergence between them is the
// interesting signal (and what BAD_CURRENT_SENSOR reports), which is exactly
// why they share one chart rather than getting one each.
const { points, version } = useTelemetryHistory({
    positive: (f) => f.positive_current,
    negative: (f) => f.negative_current,
})

// `points` arrays are markRaw and mutated in place, so `version` is the only
// reactive thing to depend on here — see useTelemetryHistory.
const series = computed(() => {
    void version.value
    return [
        { label: 'Positive pole', color: 'series1' as const, data: points.positive },
        { label: 'Negative pole', color: 'series2' as const, data: points.negative },
    ]
})

const isEmpty = computed(() => {
    void version.value
    return points.positive.length === 0 && points.negative.length === 0
})
</script>

<template>
    <TimeSeries :series="series" :is-empty="isEmpty" empty-message="Waiting for current data…" />
</template>
