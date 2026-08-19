<script setup lang="ts">
defineProps<{
    title: string
    rows: { key: string; label: string; ok: boolean | undefined }[]
    suppressed?: boolean
    suppressedNote?: string
}>()
</script>

<template>
    <div class="panel" :class="{ suppressed }">
        <h3>
            {{ title }}
            <span v-if="suppressed && suppressedNote" class="suppressed-note">{{
                suppressedNote
            }}</span>
        </h3>
        <ul class="checklist">
            <li
                v-for="row in rows"
                :key="row.key"
                :class="{ met: row.ok, unknown: row.ok === undefined }"
            >
                <span class="icon">{{ row.ok === undefined ? '–' : row.ok ? '✓' : '✕' }}</span>
                {{ row.label }}
            </li>
        </ul>
    </div>
</template>

<style scoped>
.panel {
    background: var(--surface-card);
    border: 1px solid var(--border);
    border-radius: 0.375rem;
    padding: 0.875rem 1rem;
    transition: opacity 0.15s ease;
}

.panel.suppressed {
    opacity: 0.5;
}

h3 {
    font-size: 0.75rem;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    color: var(--ink-muted);
    margin: 0 0 0.625rem;
    display: flex;
    align-items: center;
    gap: 0.5rem;
}

.suppressed-note {
    font-size: 0.65625rem;
    font-weight: 600;
    text-transform: none;
    letter-spacing: 0;
    color: var(--status-warning-text);
    background: color-mix(in srgb, var(--status-warning) 16%, var(--surface-card));
    border-radius: 0.25rem;
    padding: 0.0625rem 0.375rem;
}

.checklist {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    row-gap: 0.5rem;
}

.checklist li {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    font-size: 0.8125rem;
    color: var(--ink-secondary);
}

.checklist li.met {
    color: var(--ink-muted);
}

.icon {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    width: 1.125rem;
    height: 1.125rem;
    line-height: 1;
    font-size: 0.875rem;
    font-weight: 700;
}

.checklist li.met .icon {
    color: var(--status-good-text);
}
.checklist li:not(.met):not(.unknown) .icon {
    color: var(--status-critical-text);
}
.checklist li.unknown .icon {
    color: var(--ink-muted);
}
</style>
