<script setup lang="ts">
import { computed } from 'vue'
import TimeSeries from '@/components/TimeSeries.vue'
import { useRpmHistory } from '@/composables/useRpmHistory'

const { samples } = useRpmHistory()

const series = computed(() => [
    {
        label: 'Motor RPM',
        color: 'series1' as const,
        data: samples.map((s) => ({ x: s.t, y: s.rpm })),
    },
])
</script>

<template>
    <TimeSeries
        :series="series"
        :is-empty="samples.length === 0"
        empty-message="Waiting for RPM data…"
        :show-legend="false"
    />
</template>
