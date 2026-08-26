<script setup lang="ts">
import { computed } from 'vue'
import TimeSeries from '@/components/TimeSeries.vue'
import { useVoltageHistory } from '@/composables/useVoltageHistory'

const { samples } = useVoltageHistory()

const series = computed(() => [
    {
        label: 'Precharge Rail',
        color: 'series1' as const,
        stepped: true,
        data: samples
            .filter((s) => s.prchg !== undefined)
            .map((s) => ({ x: s.t, y: s.prchg as number })),
    },
    {
        label: 'Tractive System',
        color: 'series2' as const,
        stepped: true,
        data: samples.filter((s) => s.ts !== undefined).map((s) => ({ x: s.t, y: s.ts as number })),
    },
])
</script>

<template>
    <TimeSeries
        :series="series"
        :is-empty="samples.length === 0"
        empty-message="Waiting for precharge voltage data…"
        :y-step-size="1"
    />
</template>
