<script setup lang="ts">
import type { Severity } from '@/domain/severity'
defineProps<{
    minLabel: string
    minSeverity?: Severity
    maxLabel: string
    maxSeverity?: Severity
}>()
</script>

<template>
    <span class="ever-range">
        <span class="arrow-stat" :data-severity="minSeverity"> <span>↓</span>{{ minLabel }} </span>
        <span class="arrow-stat" :data-severity="maxSeverity"> <span>↑</span>{{ maxLabel }} </span>
    </span>
</template>

<style scoped>
.ever-range {
    display: inline-flex;
    gap: var(--gap-inline);
}

.arrow-stat {
    display: inline-flex;
    align-items: center;
    gap: var(--gap-tight);
    font-size: 0.75rem;
    color: var(--ink-muted);
    font-variant-numeric: tabular-nums;
}
.arrow-stat[data-severity='good'] {
    color: var(--status-good-text);
}
.arrow-stat[data-severity='warning'] {
    color: var(--status-warning-text);
}
.arrow-stat[data-severity='critical'] {
    color: var(--status-critical-text);
}
</style>
