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
        /**
         * Whether this component *has* its own online flag at all, distinct
         * from `online`'s current value — `online` reads `undefined` both
         * when a component has no flag of its own (rear distribution) and,
         * transiently, when a component that *does* have one just hasn't had
         * a frame yet (e.g. on first load with no connection). Only the
         * former should suppress the timing sub-line permanently.
         */
        hasOwnSignal?: boolean
    }>(),
    // Vue casts an absent Boolean-typed prop to `false` unless it has an
    // explicit default — without this, "no signal" and "signal says
    // offline" would be indistinguishable from `props.online` alone.
    { online: undefined, hasOwnSignal: false },
)

const { frame } = useTelemetry()
const { status, relativeText } = useLastSeen(() => props.online)

// `status` is still binary (online/offline) — `fault` isn't wired up to any
// real logic yet, see PLAN.md. Mapped through `AccentTile`'s generic
// good/warning/critical severity vocabulary once it is.
const severity = computed(() => (status.value === 'online' ? 'good' : 'critical'))

const statusLabel = computed(() => (status.value === 'online' ? 'Online' : 'Offline'))
const lastSeenText = computed(() => `last seen ${relativeText.value}`)

// Only the overall link tile (rear distribution — no online flag of its own)
// has a timing signal to show — other components only ever report
// online/offline, with no uptime or "last seen" of their own.
const hasTiming = computed(() => !props.hasOwnSignal)
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
