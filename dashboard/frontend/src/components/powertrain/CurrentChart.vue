<script setup lang="ts">
import { computed } from 'vue'
import TimeSeries from '@/components/TimeSeries.vue'
import { useTelemetryHistory } from '@/composables/useTelemetryHistory'

const { points, version } = useTelemetryHistory({
    desired: (f) => f.desired_motor_current,
    actual: (f) => f.motor_current,
})

const series = computed(() => {
    void version.value
    return [
        { label: 'Desired (APPS)', color: 'series1' as const, data: points.desired },
        { label: 'Actual (motor)', color: 'series2' as const, data: points.actual },
    ]
})

const isEmpty = computed(() => {
    void version.value
    return points.desired.length === 0 && points.actual.length === 0
})
</script>

<template>
    <TimeSeries :series="series" :is-empty="isEmpty" empty-message="Waiting for current data…" />
</template>
