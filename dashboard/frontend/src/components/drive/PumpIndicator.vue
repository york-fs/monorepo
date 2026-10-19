<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'

// The rear runs the coolant pump whenever the TS is active (see rear.cc), so
// this is what it's told to do rather than a measured flow.
const { frame } = useTelemetry()

const on = computed(() => frame.precharge_state === 'ACTIVE')
const label = computed(() => (on.value ? 'Coolant pump on' : 'Coolant pump off'))
</script>

<template>
    <span class="pump" :class="{ on }" role="img" :aria-label="label" :title="label">
        <svg viewBox="0 0 16 16" aria-hidden="true">
            <path d="M8 1.5C8 1.5 3 7.2 3 10.3a5 5 0 0 0 10 0C13 7.2 8 1.5 8 1.5Z" />
        </svg>
        Pump
    </span>
</template>

<style scoped>
.pump {
    display: inline-flex;
    align-items: center;
    gap: var(--gap-tight);
    font-size: 0.75rem;
    font-weight: 550;
    text-transform: uppercase;
    color: var(--ink-muted);
}

svg {
    width: 1.125rem;
    height: 1.125rem;
    fill: none;
    stroke: var(--ink-muted);
    stroke-width: 1.25;
}
.pump.on {
    color: var(--series-1);
}
.pump.on svg {
    fill: var(--series-1);
    stroke: var(--series-1);
}
</style>
