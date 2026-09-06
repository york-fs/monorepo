<script setup lang="ts">
import { computed } from 'vue'
import TimeSeries from '@/components/TimeSeries.vue'
import { useTelemetryHistory } from '@/composables/useTelemetryHistory'

const { points, version } = useTelemetryHistory({ rpm: (f) => f.motor_rpm })

// `points` arrays are markRaw and mutated in place, so nothing about them is
// reactive — `version` is the only thing that changes when new samples land,
// hence reading it here to make these computeds depend on it. See
// useTelemetryHistory.
const series = computed(() => {
    void version.value
    return [{ label: 'Motor RPM', color: 'series1' as const, data: points.rpm }]
})

const isEmpty = computed(() => {
    void version.value
    return points.rpm.length === 0
})
</script>

<template>
    <TimeSeries
        :series="series"
        :is-empty="isEmpty"
        empty-message="Waiting for RPM data…"
        :show-legend="false"
    />
</template>
