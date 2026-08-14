<script setup lang="ts">
defineProps<{
    name: string
    severity?: 'good' | 'warning' | 'critical'
}>()
</script>

<template>
    <div class="tile" :data-severity="severity">
        <span class="name">{{ name }}</span>
        <span class="value"><slot /></span>
        <div class="sub"><slot name="sub" /></div>
    </div>
</template>

<style scoped>
.tile {
    background: var(--surface-card);
    border: 1px solid var(--border);
    border-left-width: 0.25rem;
    border-radius: 0.375rem;
    padding: 0.875rem 1.125rem;
    display: grid;
    grid-template-rows: auto auto auto;
    gap: 0.25rem;
}
.tile[data-severity='good'] {
    border-left-color: var(--status-good);
}
.tile[data-severity='warning'] {
    border-left-color: var(--status-warning);
}
.tile[data-severity='critical'] {
    border-left-color: var(--status-critical);
}

.name {
    font-size: 0.6875rem;
    color: var(--ink-muted);
    text-transform: uppercase;
    letter-spacing: 0.04em;
}

.value {
    font-size: 1.375rem;
    font-weight: 650;
    letter-spacing: -0.01em;
    font-variant-numeric: tabular-nums;
}
.tile[data-severity='good'] .value {
    color: var(--status-good-text);
}
.tile[data-severity='warning'] .value {
    color: var(--status-warning-text);
}
.tile[data-severity='critical'] .value {
    color: var(--status-critical-text);
}

/* Reserves the secondary row's height even when a consumer has nothing to
   put in it, so tiles sharing a row stay the same height regardless of
   whether they use the `sub` slot (see rear distribution's uptime line in
   ConnectionStatusTile). */
.sub {
    min-height: 1.1875rem;
}
</style>
