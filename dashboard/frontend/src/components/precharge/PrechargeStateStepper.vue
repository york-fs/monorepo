<script setup lang="ts">
import { computed } from 'vue'
import type { PrechargeState } from '@/telemetry'
import { PRECHARGE_STATES, PRECHARGE_STATE_LABELS } from '@/domain/precharge'

const props = defineProps<{
    state?: PrechargeState
}>()

const currentIndex = computed(() => (props.state ? PRECHARGE_STATES.indexOf(props.state) : -1))
</script>

<template>
    <ol class="stepper">
        <li
            v-for="(s, i) in PRECHARGE_STATES"
            :key="s"
            class="step"
            :class="{ current: i === currentIndex }"
        >
            <span v-if="i > 0" class="connector" />
            <span class="marker" />
            <span class="label">{{ PRECHARGE_STATE_LABELS[s] }}</span>
        </li>
    </ol>
</template>

<style scoped>
.stepper {
    display: flex;
    align-items: center;
    list-style: none;
    margin: 0;
    padding: 0;
}

.step {
    display: flex;
    align-items: center;
    gap: 0.5rem;
}

.connector {
    width: 1.25rem;
    height: 1px;
    background: var(--border);
    margin: 0 0.625rem;
    flex-shrink: 0;
}

/* Horizontal connectors don't degrade gracefully when the row wraps, so
   switch to a vertical list instead of letting it wrap on narrow screens. */
@media (max-width: 40em) {
    .stepper {
        flex-direction: column;
        align-items: flex-start;
        gap: 0.5rem;
    }

    .connector {
        display: none;
    }
}

.marker {
    width: 0.5rem;
    height: 0.5rem;
    border-radius: 50%;
    background: var(--border);
    flex-shrink: 0;
}

.label {
    font-size: 0.78125rem;
    color: var(--ink-muted);
    white-space: nowrap;
}

.step.current .marker {
    background: var(--ink-primary);
}

.step.current .label {
    color: var(--ink-primary);
    font-weight: 650;
}
</style>
