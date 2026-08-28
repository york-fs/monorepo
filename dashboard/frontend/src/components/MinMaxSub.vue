<script setup lang="ts">
defineProps<{
    minLabel: string
    minSeverity?: 'good' | 'warning' | 'critical'
    maxLabel: string
    maxSeverity?: 'good' | 'warning' | 'critical'
}>()
</script>

<template>
    <span class="ever-range">
        <span class="arrow-stat" :data-severity="minSeverity">
            <span class="arrow">↓</span>{{ minLabel }}
        </span>
        <span class="arrow-stat" :data-severity="maxSeverity">
            <span class="arrow">↑</span>{{ maxLabel }}
        </span>
    </span>
</template>

<style scoped>
.ever-range {
    display: inline-flex;
    gap: 0.75rem;
}

.arrow-stat {
    display: inline-flex;
    align-items: center;
    gap: 0.1875rem;
    font-size: 0.6875rem;
    font-weight: 600;
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

.arrow {
    font-size: 0.75rem;
    line-height: 1;
}
</style>
