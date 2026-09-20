<template>
    <span class="badge"><slot /></span>
</template>

<style scoped>
.badge {
    font-size: 0.75rem;
    padding: 0.25rem 0.5rem;
    border-radius: var(--radius-badge);
    background: color-mix(in srgb, var(--status-warning) 16%, var(--surface-tile));
    color: var(--status-warning-text);
    border: 1px solid var(--border);
}
</style>
