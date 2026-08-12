<script setup lang="ts">
import { computed } from 'vue'
import type { PrechargeRelay } from '@/types/telemetry'
import { PRECHARGE_RELAY_LABELS } from '@/domain/precharge'

const props = defineProps<{
    relays?: readonly PrechargeRelay[]
}>()

const ALL_RELAYS = Object.keys(PRECHARGE_RELAY_LABELS) as PrechargeRelay[]

const rows = computed(() =>
    ALL_RELAYS.map((relay) => ({
        relay,
        label: PRECHARGE_RELAY_LABELS[relay],
        closed: (props.relays ?? []).includes(relay),
    })),
)
</script>

<template>
    <ul class="relays">
        <li v-for="row in rows" :key="row.relay" class="relay">
            <span class="label">{{ row.label }}</span>
            <span class="state" :class="{ open: !row.closed }">{{
                row.closed ? 'Closed' : 'Open'
            }}</span>
        </li>
    </ul>
</template>

<style scoped>
.relays {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    row-gap: 0.375rem;
}

.relay {
    display: grid;
    grid-template-columns: 1fr auto;
    align-items: center;
    column-gap: 0.75rem;
    font-size: 0.8125rem;
}

.label {
    color: var(--ink-secondary);
}

.state {
    font-weight: 600;
    text-align: right;
}

.state.open {
    color: var(--ink-muted);
}
</style>
