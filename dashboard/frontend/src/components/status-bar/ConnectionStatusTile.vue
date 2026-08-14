<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import AccentTile from '@/components/AccentTile.vue'
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

// `status` is still binary (online/offline) — `fault` isn't wired up to any
// real logic yet, see PLAN.md. Mapped through `AccentTile`'s generic
// good/warning/critical severity vocabulary once it is.
const severity = computed(() => (status.value === 'online' ? 'good' : 'critical'))

const statusLabel = computed(() => (status.value === 'online' ? 'Online' : 'Offline'))
const lastSeenText = computed(() => `last seen ${relativeText.value}`)

// Only the overall link tile (no `online` prop of its own) has a timing
// signal to show — other components only ever report online/offline, with
// no uptime or "last seen" of their own.
const hasTiming = computed(() => props.online === undefined)
</script>

<template>
    <AccentTile :name="props.name" :severity="severity">
        {{ statusLabel }}
        <template #sub>
            <UptimeDisplay v-if="hasTiming && status === 'online'" :ms="frame.uptime" prefix="up" />
            <UptimeDisplay v-else-if="hasTiming" :text="lastSeenText" />
        </template>
    </AccentTile>
</template>
