<script setup lang="ts">
import { computed } from 'vue'
import TimeSeries from '@/components/TimeSeries.vue'
import { useTelemetryHistory } from '@/composables/useTelemetryHistory'

const { points, version } = useTelemetryHistory({
    prchg: (f) => f.precharge_prchg_voltage,
    ts: (f) => f.precharge_ts_voltage,
})

// `points` arrays are markRaw and mutated in place, so nothing about them is
// reactive — `version` is the only thing that changes when new samples land,
// hence reading it here to make these computeds depend on it. See
// useTelemetryHistory.
const series = computed(() => {
    void version.value
    return [
        { label: 'Precharge Rail', color: 'series1' as const, stepped: true, data: points.prchg },
        { label: 'Tractive System', color: 'series2' as const, stepped: true, data: points.ts },
    ]
})

const isEmpty = computed(() => {
    void version.value
    return points.prchg.length === 0 && points.ts.length === 0
})
</script>

<template>
    <TimeSeries
        :series="series"
        :is-empty="isEmpty"
        empty-message="Waiting for precharge voltage data…"
    />
</template>
