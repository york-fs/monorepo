<script setup lang="ts">
import { computed } from 'vue'
import Tile from '@/components/Tile.vue'

// A running total (see useRunningTotal), with when it started counting and a
// way to start it again.
const props = defineProps<{ title: string; value: string; since?: Date }>()
defineEmits<{ reset: [] }>()

const sinceLabel = computed(() =>
    props.since === undefined
        ? 'Not counting yet'
        : `Since ${props.since.toLocaleTimeString([], { hour12: false })}`,
)
</script>

<template>
    <Tile :title="title" :value="value">
        <template #sub>
            <div class="sub">
                <span>{{ sinceLabel }}</span>
                <button type="button" class="reset" @click="$emit('reset')">Reset</button>
            </div>
        </template>
    </Tile>
</template>

<style scoped>
.sub {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: var(--gap-items);
    font-size: 0.8125rem;
    color: var(--ink-muted);
}

.reset {
    padding: 0;
    font: inherit;
    color: var(--ink-secondary);
    background: none;
    border: none;
    text-decoration: underline;
    cursor: pointer;
}
</style>
