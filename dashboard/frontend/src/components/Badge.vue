<script setup lang="ts">
// The small amber "what you're looking at isn't the real thing" notice —
// Section's "Stale" banner and HomeView's demo-mode label, which were
// identical but for their text.
</script>

<template>
    <span class="badge"><slot /></span>
</template>

<style scoped>
.badge {
    font-size: 0.75rem;
    font-weight: 600;
    padding: 0.125rem 0.5rem;
    border-radius: var(--radius-badge);
    /* A tinted background rather than a flat --status-warning fill: in dark
       mode --status-warning-text *is* --status-warning, so amber-on-amber
       would render the label invisible. */
    background: color-mix(in srgb, var(--status-warning) 16%, var(--surface-tile));
    color: var(--status-warning-text);
    border: 1px solid var(--border);
}
</style>
