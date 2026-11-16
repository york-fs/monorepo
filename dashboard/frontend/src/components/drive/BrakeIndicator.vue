<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'

// The brake switch as the rear reads it, which is also what lights the brake
// light and cuts the throttle.
const { frame } = useTelemetry()

const on = computed(() => frame.brake_pressed === true)
const label = computed(() => (on.value ? 'Brake pressed' : 'Brake released'))
</script>

<template>
    <span class="brake" :class="{ on }" role="img" :aria-label="label" :title="label">Brake</span>
</template>

<style scoped>
.brake {
    font-size: 0.75rem;
    font-weight: 550;
    text-transform: uppercase;
    color: var(--ink-muted);
}
.brake.on {
    color: var(--status-critical-text);
}
</style>
