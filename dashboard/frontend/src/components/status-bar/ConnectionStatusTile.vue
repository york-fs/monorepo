<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import UptimeDisplay from '@/components/status-bar/UptimeDisplay.vue'
import { useLastSeen } from '@/composables/useLastSeen'

const props = withDefaults(
    defineProps<{
        name: string
        /** This component's own online signal, if it has one — see `useLastSeen`. */
        online?: boolean
    }>(),
    // Vue casts an absent Boolean-typed prop to `false` unless it has an
    // explicit default — without this, "no signal" and "signal says
    // offline" would be indistinguishable from `props.online` alone.
    { online: undefined },
)

const { frame } = useTelemetry()
const { status, relativeText } = useLastSeen(() => props.online)

const statusLabel = computed(() => (status.value === 'online' ? 'Online' : 'Offline'))
const lastSeenText = computed(() => `last seen ${relativeText.value}`)

// Only the overall link tile (no `online` prop of its own) has a timing
// signal to show — other components only ever report online/offline, with
// no uptime or "last seen" of their own.
const hasTiming = computed(() => props.online === undefined)
</script>

<template>
    <div class="tile" :data-status="status">
        <span class="name">{{ props.name }}</span>
        <span class="status-text">{{ statusLabel }}</span>
        <div class="timing">
            <UptimeDisplay v-if="hasTiming && status === 'online'" :ms="frame.uptime" prefix="up" />
            <UptimeDisplay v-else-if="hasTiming" :text="lastSeenText" />
        </div>
    </div>
</template>

<style scoped>
.tile {
    background: var(--surface-card);
    border: 1px solid var(--border);
    border-left-width: 0.25rem;
    border-radius: 0.375rem;
    padding: 0.875rem 1.125rem;
    display: grid;
    grid-template-rows: auto auto auto;
    gap: 0.25rem;
}

/* Reserves the uptime/last-seen line's height on every tile, even the
   ones with nothing to show there, so a tile with timing data (currently
   only rear distribution/overall link) doesn't end up taller than its
   siblings. */
.timing {
    min-height: 1.1875rem;
}

.tile[data-status='online'] {
    --status-text-color: var(--status-good-text);
    border-left-color: var(--status-good);
}
.tile[data-status='offline'] {
    --status-text-color: var(--status-critical-text);
    border-left-color: var(--status-critical);
}
/* Not wired up to any real logic yet — `status` is still binary
   (`online`/`offline`). Styling only, ready for when a component can
   report "online but flagging a fault to investigate". */
.tile[data-status='fault'] {
    --status-text-color: var(--status-warning-text);
    border-left-color: var(--status-warning);
}

.name {
    font-size: 0.6875rem;
    color: var(--ink-muted);
    text-transform: uppercase;
    letter-spacing: 0.04em;
}

.status-text {
    font-size: 1.375rem;
    font-weight: 650;
    letter-spacing: -0.01em;
    color: var(--status-text-color);
}
</style>
