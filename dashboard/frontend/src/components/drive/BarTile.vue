<script setup lang="ts">
import { computed } from 'vue'
import Tile from '@/components/Tile.vue'

// A reading out of a fixed maximum, with a bar under the value showing how
// much of it is in use: motor current against the inverter's limit, pedal
// travel against full travel.
const props = defineProps<{
    name: string
    value?: number
    max: number
    /** Formats both the value and the maximum it's shown against. */
    format: (value: number) => string
    large?: boolean
}>()

const fill = computed(() =>
    props.value === undefined ? 0 : Math.min(1, Math.abs(props.value) / props.max),
)
</script>

<template>
    <Tile :title="name" :value="value === undefined ? '—' : format(value)" :large="large">
        <template #sub>
            <div class="gauge">
                <div
                    class="bar"
                    role="meter"
                    :aria-label="name"
                    :aria-valuenow="value"
                    aria-valuemin="0"
                    :aria-valuemax="max"
                >
                    <div class="fill" :style="{ width: `${fill * 100}%` }" />
                </div>
                <span class="max">of {{ format(max) }}</span>
            </div>
        </template>
    </Tile>
</template>

<style scoped>
.gauge {
    display: flex;
    align-items: center;
    gap: var(--gap-items);
}

.bar {
    flex: 1;
    height: 0.5rem;
    border-radius: 0.25rem;
    background: var(--border);
    overflow: hidden;
}

.fill {
    height: 100%;
    background: var(--series-1);
    transition: width 0.1s linear;
}

.max {
    font-size: 0.8125rem;
    color: var(--ink-muted);
    font-variant-numeric: tabular-nums;
}
</style>
