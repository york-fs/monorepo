<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { shutdownOpenCauseLabel, shutdownOpenCauseSeverity } from '@/domain/shutdown'
import type { Severity } from '@/domain/severity'

// The handful of readings worth seeing from every page: whether the link is
// up, whether TS and RTD are on, and what's holding shutdown open.
const { frame, status } = useTelemetry()

interface StripItem {
    name: string
    value: string
    severity?: Severity
}

const tsItem = computed<StripItem>(() => {
    const state = frame.precharge_state
    if (state === undefined) return { name: 'TS', value: '—' }
    if (state === 'ACTIVE') return { name: 'TS', value: 'On', severity: 'good' }
    if (state === 'PRECHARGE' || state === 'PRECHARGE_HOLD') {
        return { name: 'TS', value: 'Precharging', severity: 'warning' }
    }
    return { name: 'TS', value: 'Off' }
})

const rtdItem = computed<StripItem>(() => {
    const flags = frame.rtd_prevention_flags
    if (flags === undefined) return { name: 'RTD', value: '—' }
    return flags.length === 0
        ? { name: 'RTD', value: 'On', severity: 'good' }
        : { name: 'RTD', value: 'Off' }
})

const shutdownItem = computed<StripItem>(() => {
    const cause = frame.shutdown_open_cause
    return {
        name: 'Shutdown',
        value: cause === 'NONE' ? 'Closed' : shutdownOpenCauseLabel(cause),
        severity: shutdownOpenCauseSeverity(cause),
    }
})

const linkItem = computed<StripItem>(() =>
    status.value === 'online'
        ? { name: 'Link', value: 'Online', severity: 'good' }
        : { name: 'Link', value: 'Offline', severity: 'critical' },
)

// Everything but the link item is the car's last word, so it dims once the
// link goes quiet, same as a `Section`.
const carItems = computed(() => [tsItem.value, rtdItem.value, shutdownItem.value])
</script>

<template>
    <dl class="strip">
        <div class="item" :data-severity="linkItem.severity">
            <dt>{{ linkItem.name }}</dt>
            <dd>{{ linkItem.value }}</dd>
        </div>
        <div
            v-for="item in carItems"
            :key="item.name"
            class="item"
            :class="{ dimmed: status === 'offline' }"
            :data-severity="item.severity"
        >
            <dt>{{ item.name }}</dt>
            <dd>{{ item.value }}</dd>
        </div>
    </dl>
</template>

<style scoped>
.strip {
    display: flex;
    flex-wrap: wrap;
    gap: var(--gap-rows);
    margin: 0;
}

.item {
    display: inline-flex;
    align-items: center;
    gap: var(--gap-items);
    padding: 0.25rem 0.625rem;
    font-size: 0.875rem;
    background: var(--surface-tile);
    border: 1px solid var(--border);
    border-radius: 1rem;
}

.item::before {
    content: '';
    width: 0.5rem;
    height: 0.5rem;
    border-radius: 50%;
    background: var(--ink-muted);
}
.item[data-severity='good']::before {
    background: var(--status-good);
}
.item[data-severity='warning']::before {
    background: var(--status-warning);
}
.item[data-severity='critical']::before {
    background: var(--status-critical);
}

.item.dimmed {
    opacity: 0.5;
}

dt {
    font-size: 0.75rem;
    font-weight: 550;
    text-transform: uppercase;
    color: var(--ink-muted);
}

dd {
    margin: 0;
    font-weight: 600;
}
.item[data-severity='critical'] dd {
    color: var(--status-critical-text);
}
</style>
