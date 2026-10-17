<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref, useId } from 'vue'
import AutoGrid from '@/components/AutoGrid.vue'
import EventLogList from '@/components/events/EventLogList.vue'
import LastEventTile from '@/components/events/LastEventTile.vue'
import { useEventLog } from '@/composables/useEventLog'
import type { EventCategory, LogEvent } from '@/domain/events'

// Categories a "last fault" can come from — TS, RTD and shutdown have tiles
// of their own.
const FAULT_CATEGORIES = new Set<EventCategory>(['bms', 'inverter', 'precharge', 'online', 'fuse'])

const { events, available, loadError } = useEventLog()

function latest(match: (event: LogEvent) => boolean): LogEvent | undefined {
    for (let i = events.value.length - 1; i >= 0; i--) {
        if (match(events.value[i]!)) return events.value[i]
    }
    return undefined
}

const lastTsDrop = computed(() =>
    latest((e) => e.category === 'ts' && (e.kind === 'deactivated' || e.kind === 'aborted')),
)
const lastRtdLoss = computed(() => latest((e) => e.category === 'rtd' && e.kind === 'deactivated'))
const lastShutdown = computed(() =>
    latest((e) => e.category === 'shutdown' && (e.kind === 'raised' || e.kind === 'blip')),
)
const lastFault = computed(() =>
    latest(
        (e) =>
            (e.kind === 'raised' || e.kind === 'blip') &&
            e.severity === 'fault' &&
            FAULT_CATEGORIES.has(e.category),
    ),
)

// One coarse ticker for the "x minutes ago" text in every tile.
const now = ref(Date.now())
let timer: ReturnType<typeof setInterval> | undefined
onMounted(() => {
    timer = setInterval(() => (now.value = Date.now()), 5000)
})
onUnmounted(() => clearInterval(timer))

const headingId = useId()
</script>

<template>
    <!-- Not a `Section`: those dim when the telemetry link drops, and the log
         is most useful exactly then. -->
    <section class="section" :aria-labelledby="headingId">
        <h2 :id="headingId">Event log</h2>
        <p v-if="!available" class="note">
            The event log is recorded by the dashboard server, so it isn't available in demo mode.
        </p>
        <template v-else>
            <p v-if="loadError" class="note">{{ loadError }}</p>
            <AutoGrid min="16rem">
                <LastEventTile title="Last TS deactivation" :event="lastTsDrop" :now="now" />
                <LastEventTile title="Last RTD loss" :event="lastRtdLoss" :now="now" />
                <LastEventTile title="Last shutdown open" :event="lastShutdown" :now="now" />
                <LastEventTile title="Last fault" :event="lastFault" :now="now" show-event-title />
            </AutoGrid>
            <EventLogList />
        </template>
    </section>
</template>

<style scoped>
.section {
    display: grid;
    gap: var(--gap-blocks);
}

.note {
    margin: 0;
    font-size: 0.8125rem;
    color: var(--ink-muted);
}
</style>
