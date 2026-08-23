<script setup lang="ts">
import SeverityCard from '@/components/SeverityCard.vue'

defineProps<{
    title: string
    rows: { key: string; label: string; ok: boolean | undefined }[]
    severity?: 'good' | 'warning' | 'critical'
}>()
</script>

<template>
    <SeverityCard class="panel" :severity="severity">
        <h3>{{ title }}</h3>
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
    </SeverityCard>
</template>

<style scoped>
.panel {
    padding: 0.875rem 1rem;
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
