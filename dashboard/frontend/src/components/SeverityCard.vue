<script setup lang="ts">
// Shared "severity-accented card" shell — background/border plus the
// coloured border-left keyed off severity — factored out of AccentTile and
// PreventionChecklistPanel, which used to each define an identical copy of
// this. Deliberately only the shell: each consumer still owns its own
// padding/layout/content styling, since AccentTile's single-big-value shape
// and the checklist panels' list shape have nothing else in common.
defineProps<{
    severity?: 'good' | 'warning' | 'critical'
}>()
</script>

<template>
    <div class="card" :data-severity="severity">
        <slot />
    </div>
</template>

<style scoped>
.card {
    background: var(--surface-card);
    border: 1px solid var(--border);
    border-left-width: 0.25rem;
    border-radius: 0.375rem;
}
.card[data-severity='good'] {
    border-left-color: var(--status-good);
}
.card[data-severity='warning'] {
    border-left-color: var(--status-warning);
}
.card[data-severity='critical'] {
    border-left-color: var(--status-critical);
}
</style>
