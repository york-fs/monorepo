<script setup lang="ts">
import type { Severity } from '@/domain/severity'
import Tile from '@/components/Tile.vue'

defineProps<{
    title: string
    rows: { flag: string; label: string; ok: boolean | undefined }[]
    severity?: Severity
}>()
</script>

<template>
    <Tile :title="title" :level="4" accent :severity="severity">
        <ul class="checklist">
            <li
                v-for="row in rows"
                :key="row.flag"
                :class="{ met: row.ok, unknown: row.ok === undefined }"
            >
                <span class="icon">{{ row.ok === undefined ? '–' : row.ok ? '✓' : '✕' }}</span>
                {{ row.label }}
            </li>
        </ul>
    </Tile>
</template>

<style scoped>
.checklist {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    row-gap: var(--gap-rows);
    font-size: 0.8125rem;
}

.checklist li {
    display: flex;
    align-items: center;
    gap: var(--gap-rows);
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
