<script setup lang="ts">
import { useDebugMode } from '@/composables/useDebugMode'

// A switch rather than a checkbox: it changes what the page shows straight
// away, with nothing to submit.
const { debugMode } = useDebugMode()
</script>

<template>
    <button
        type="button"
        role="switch"
        class="toggle"
        :aria-checked="debugMode"
        @click="debugMode = !debugMode"
    >
        <span class="track"><span class="knob" /></span>
        Debug
    </button>
</template>

<style scoped>
.toggle {
    display: inline-flex;
    align-items: center;
    gap: var(--gap-items);
    padding: 0.25rem 0.5rem;
    font: inherit;
    font-size: 0.8125rem;
    color: var(--ink-secondary);
    background: transparent;
    border: 1px solid var(--border);
    border-radius: var(--radius-badge);
    cursor: pointer;
}

.toggle[aria-checked='true'] {
    color: var(--ink-primary);
}

.track {
    position: relative;
    width: 1.75rem;
    height: 1rem;
    border-radius: 0.5rem;
    box-shadow: inset 0 0 0 1px var(--ink-muted);
    transition: background-color 0.15s;
}

.toggle[aria-checked='true'] .track {
    background: var(--series-1);
    box-shadow: none;
}

.knob {
    position: absolute;
    top: 0.125rem;
    left: 0.125rem;
    width: 0.75rem;
    height: 0.75rem;
    border-radius: 50%;
    background: var(--ink-muted);
    transition: transform 0.15s;
}

.toggle[aria-checked='true'] .knob {
    background: var(--surface-tile);
    transform: translateX(0.75rem);
}
</style>
