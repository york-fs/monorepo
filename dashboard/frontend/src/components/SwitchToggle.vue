<script setup lang="ts">
// A switch rather than a checkbox: it changes what the dash does straight
// away, with nothing to submit. The default slot is its label.
const enabled = defineModel<boolean>({ required: true })
</script>

<template>
    <button
        type="button"
        role="switch"
        class="toggle"
        :aria-checked="enabled"
        @click="enabled = !enabled"
    >
        <span class="track"><span class="knob" /></span>
        <slot />
    </button>
</template>

<style scoped>
.toggle {
    display: inline-flex;
    align-items: center;
    gap: var(--gap-items);
    padding: 0;
    font: inherit;
    font-size: 0.75rem;
    color: var(--ink-muted);
    background: transparent;
    border: none;
    cursor: pointer;
}

.toggle[aria-checked='true'] {
    color: var(--ink-secondary);
}

.track {
    position: relative;
    width: 1.375rem;
    height: 0.75rem;
    border-radius: 0.375rem;
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
    width: 0.5rem;
    height: 0.5rem;
    border-radius: 50%;
    background: var(--ink-muted);
    transition: transform 0.15s;
}

.toggle[aria-checked='true'] .knob {
    background: var(--surface-tile);
    transform: translateX(0.625rem);
}
</style>
