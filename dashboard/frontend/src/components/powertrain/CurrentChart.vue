<script setup lang="ts">
import { computed } from 'vue'
import TimeSeries from '@/components/TimeSeries.vue'
import { useTelemetryHistory } from '@/composables/useTelemetryHistory'

const { points, version } = useTelemetryHistory({
    desired: (f) => f.desired_motor_current,
    commanded: (f) => f.commanded_motor_current,
    actual: (f) => f.motor_current,
})

// `points` arrays are markRaw and mutated in place, so nothing about them is
// reactive — `version` is the only thing that changes when new samples land,
// hence reading it here to make these computeds depend on it. See
// useTelemetryHistory.
const series = computed(() => {
    void version.value
    return [
        // The pedal's demand, then what the rear sent once its launch soft start
        // had limited it, then what the motor drew.
        { label: 'Pedal demand', color: 'series3' as const, data: points.desired },
        { label: 'Commanded (rear)', color: 'series1' as const, data: points.commanded },
        { label: 'Actual (motor)', color: 'series2' as const, data: points.actual },
    ]
})

const isEmpty = computed(() => {
    void version.value
    return (
        points.desired.length === 0 &&
        points.commanded.length === 0 &&
        points.actual.length === 0
    )
})
</script>

<template>
    <TimeSeries :series="series" :is-empty="isEmpty" empty-message="Waiting for current data…" />
</template>
