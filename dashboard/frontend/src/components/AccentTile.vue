<script setup lang="ts">
import type { Severity } from '@/domain/severity'
import Tile from '@/components/Tile.vue'

defineProps<{
    name: string
    severity?: Severity
}>()
</script>

<template>
    <Tile class="value-tile" accent :severity="severity">
        <span class="tile-label">{{ name }}</span>
        <span class="value"><slot /></span>
        <div class="sub"><slot name="sub" /></div>
    </Tile>
</template>

<style scoped>
.value-tile {
    display: grid;
    gap: var(--gap-tight);
}

.value {
    font-size: 1.375rem;
    font-weight: 650;
    letter-spacing: -0.01em;
    font-variant-numeric: tabular-nums;
}
.value-tile[data-severity='good'] .value {
    color: var(--status-good-text);
}
.value-tile[data-severity='warning'] .value {
    color: var(--status-warning-text);
}
.value-tile[data-severity='critical'] .value {
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
