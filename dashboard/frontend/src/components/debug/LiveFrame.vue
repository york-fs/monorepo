<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import Tile from '@/components/Tile.vue'

// Every field of the latest frame exactly as it arrived, for when a tile's
// summary isn't enough: the raw flag sets, the `_seen` sets catching blips
// between frames, and the error counters nothing else shows.
const { frame } = useTelemetry()

// Flag lists read as a list; anything nested (the per-segment fields) as JSON.
function show(value: unknown): string {
    if (value === undefined || value === null) return '—'
    if (Array.isArray(value) && value.every((item) => typeof item !== 'object')) {
        return value.length === 0 ? '—' : value.join(', ')
    }
    return typeof value === 'object' ? JSON.stringify(value) : String(value)
}

const fields = computed(() =>
    Object.entries(frame).map(([name, value]) => ({ name, value: show(value) })),
)
</script>

<template>
    <Tile title="Live frame" :level="4">
        <p v-if="fields.length === 0" class="empty">No frame received yet</p>
        <dl v-else class="fields">
            <div v-for="field in fields" :key="field.name" class="field">
                <dt>{{ field.name }}</dt>
                <dd>{{ field.value }}</dd>
            </div>
        </dl>
    </Tile>
</template>

<style scoped>
.empty {
    margin: 0;
    font-size: 0.8125rem;
    color: var(--ink-muted);
}

.fields {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(18rem, 1fr));
    gap: var(--gap-tight) var(--gap-inline);
    margin: 0;
}

.field {
    display: flex;
    justify-content: space-between;
    gap: var(--gap-items);
    padding: 0.25rem 0;
    font-size: 0.8125rem;
    border-bottom: 1px solid var(--border);
}

dt {
    color: var(--ink-muted);
    font-family: ui-monospace, monospace;
}

dd {
    margin: 0;
    text-align: right;
    font-variant-numeric: tabular-nums;
    overflow-wrap: anywhere;
}
</style>
