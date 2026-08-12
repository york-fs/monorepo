<script setup lang="ts">
import { computed } from 'vue'
import type { PrechargeErrorFlag, PrechargeState } from '@/types/telemetry'
import { PRECHARGE_FLAG_META, isFlagLive } from '@/domain/precharge'

const props = defineProps<{
    flags?: readonly PrechargeErrorFlag[]
    state?: PrechargeState
}>()

function humanize(flag: string): string {
    return flag
        .toLowerCase()
        .split('_')
        .map((word) => word[0]!.toUpperCase() + word.slice(1))
        .join(' ')
}

const KIND_ORDER = { fault: 0, waiting: 1, deactivation: 2 }

const rows = computed(() =>
    (props.flags ?? [])
        .map((flag) => ({
            flag,
            meta: PRECHARGE_FLAG_META[flag],
            live: isFlagLive(flag, props.state),
        }))
        .sort((a, b) => KIND_ORDER[a.meta.kind] - KIND_ORDER[b.meta.kind]),
)
</script>

<template>
    <div class="flags">
        <p v-if="rows.length === 0" class="empty">No flags set</p>
        <ul v-else class="list">
            <li
                v-for="row in rows"
                :key="row.flag"
                class="row"
                :class="[row.meta.kind, { latched: !row.live }]"
            >
                <span class="dot" />
                <div class="text">
                    <span class="name">
                        {{ humanize(row.flag) }}
                        <span v-if="!row.live" class="latched-tag">last attempt</span>
                    </span>
                    <span class="description">{{ row.meta.description }}</span>
                </div>
            </li>
        </ul>
    </div>
</template>

<style scoped>
.empty {
    margin: 0;
    font-size: 0.8125rem;
    color: var(--ink-muted);
}

.list {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    row-gap: 0.625rem;
}

.row {
    display: grid;
    grid-template-columns: auto 1fr;
    column-gap: 0.5rem;
}

.dot {
    width: 0.5rem;
    height: 0.5rem;
    border-radius: 50%;
    margin-top: 0.25rem;
    background: var(--ink-muted);
}

.row.fault .dot {
    background: var(--status-critical);
}

.row.latched .dot {
    opacity: 0.4;
}

.text {
    display: grid;
    gap: 0.0625rem;
}

.name {
    font-size: 0.8125rem;
    font-weight: 600;
    color: var(--ink-primary);
}

.row.fault .name {
    color: var(--status-critical-text);
}

.row.latched .name {
    color: var(--ink-secondary);
}

.latched-tag {
    font-size: 0.65625rem;
    font-weight: 500;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    color: var(--ink-muted);
    margin-left: 0.375rem;
}

.description {
    font-size: 0.75rem;
    color: var(--ink-muted);
}
</style>
