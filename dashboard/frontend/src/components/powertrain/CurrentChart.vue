<script setup lang="ts">
import { computed } from 'vue'
import TimeSeries from '@/components/TimeSeries.vue'
import { useCurrentHistory } from '@/composables/useCurrentHistory'

const { samples } = useCurrentHistory()

const series = computed(() => [
    {
        label: 'Desired (APPS)',
        color: 'series1' as const,
        data: samples
            .filter((s) => s.apps !== undefined)
            .map((s) => ({ x: s.t, y: s.apps as number })),
    },
    {
        label: 'Actual (motor)',
        color: 'series2' as const,
        data: samples
            .filter((s) => s.motor !== undefined)
            .map((s) => ({ x: s.t, y: s.motor as number })),
    },
])
</script>

<template>
    <TimeSeries
        :series="series"
        :is-empty="samples.length === 0"
        empty-message="Waiting for current data…"
    />
</template>
