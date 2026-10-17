<script setup lang="ts">
import { computed } from 'vue'
import Tile from '@/components/Tile.vue'
import EventSnapshot from '@/components/events/EventSnapshot.vue'
import { eventDescription, eventTitle, formatEventTime, type LogEvent } from '@/domain/events'
import { formatRelativeTime } from '@/utils/formatRelativeTime'

// "When did this last happen, and why" for one kind of event. Amber rather
// than red when there is one: it's history, not necessarily a live fault.
const props = defineProps<{
    title: string
    event?: LogEvent
    /** Current time, ticked by the parent so every tile shares one timer. */
    now: number
    /** Show the event's own title — for tiles that cover several codes. */
    showEventTitle?: boolean
}>()

const severity = computed(() => (props.event ? 'warning' : 'good'))
const when = computed(() =>
    props.event
        ? `${formatEventTime(props.event.time)} · ${formatRelativeTime((props.now - props.event.time) / 1000)}`
        : undefined,
)
</script>

<template>
    <Tile :title="title" :level="4" accent :severity="severity">
        <p v-if="!event" class="empty">None logged</p>
        <div v-else class="body">
            <p class="when">{{ when }}</p>
            <p v-if="showEventTitle || !event.snapshot" class="headline">
                {{ eventTitle(event) }}
            </p>
            <EventSnapshot v-if="event.snapshot" :snapshot="event.snapshot" compact />
            <p v-else-if="eventDescription(event)" class="description">
                {{ eventDescription(event) }}
            </p>
        </div>
    </Tile>
</template>

<style scoped>
.empty,
.when,
.headline,
.description {
    margin: 0;
}

.empty,
.description {
    font-size: 0.8125rem;
    color: var(--ink-muted);
}

.body {
    display: grid;
    gap: var(--gap-rows);
}

.when {
    font-size: 0.75rem;
    font-variant-numeric: tabular-nums;
    color: var(--ink-muted);
}

.headline {
    font-weight: 600;
    color: var(--status-critical-text);
}
</style>
