<script setup lang="ts">
import { computed, ref } from 'vue'
import Tile from '@/components/Tile.vue'
import EventSnapshot from '@/components/events/EventSnapshot.vue'
import { useEventLog } from '@/composables/useEventLog'
import {
    EVENT_CATEGORIES,
    EVENT_CATEGORY_LABELS,
    eventDescription,
    eventTitle,
    formatEventTime,
    type EventCategory,
} from '@/domain/events'
import { formatUptimeWithMs } from '@/utils/formatUptime'

// Rendering every row of a long session gets slow; the full history is
// always available through the download link.
const MAX_ROWS = 500

const { events, clearHistory } = useEventLog()

const hidden = ref(new Set<EventCategory>())
const showRoutine = ref(false)

function toggle(category: EventCategory) {
    const next = new Set(hidden.value)
    if (next.has(category)) next.delete(category)
    else next.add(category)
    hidden.value = next
}

const matching = computed(() =>
    events.value.filter(
        (event) =>
            !hidden.value.has(event.category) && (showRoutine.value || event.severity !== 'info'),
    ),
)

// Newest first: the thing you're usually debugging just happened.
const rows = computed(() => matching.value.slice(-MAX_ROWS).reverse())

async function clear() {
    if (window.confirm('Clear the event log? The current log file is archived, not deleted.')) {
        await clearHistory()
    }
}
</script>

<template>
    <Tile title="History" :level="4">
        <div class="toolbar">
            <div class="chips" role="group" aria-label="Categories">
                <button
                    v-for="category in EVENT_CATEGORIES"
                    :key="category"
                    type="button"
                    class="chip"
                    :aria-pressed="!hidden.has(category)"
                    @click="toggle(category)"
                >
                    {{ EVENT_CATEGORY_LABELS[category] }}
                </button>
            </div>
            <div class="actions">
                <label class="routine">
                    <input v-model="showRoutine" type="checkbox" />
                    Show routine
                </label>
                <a class="action" href="/api/events/download" download>Download</a>
                <button type="button" class="action" @click="clear">Clear</button>
            </div>
        </div>

        <p v-if="rows.length === 0" class="empty">No events match</p>
        <ol v-else class="list">
            <li v-for="event in rows" :key="event.id" class="row" :data-severity="event.severity">
                <span class="dot" />
                <div class="text">
                    <div class="line">
                        <span
                            class="time"
                            :title="`Car uptime ${formatUptimeWithMs(event.uptime / 1000)}`"
                        >
                            {{ formatEventTime(event.time) }}
                        </span>
                        <span class="category">{{ EVENT_CATEGORY_LABELS[event.category] }}</span>
                        <span class="title">{{ eventTitle(event) }}</span>
                    </div>
                    <span v-if="eventDescription(event)" class="description">
                        {{ eventDescription(event) }}
                    </span>
                    <details v-if="event.snapshot" class="details">
                        <summary>Details</summary>
                        <EventSnapshot :snapshot="event.snapshot" />
                    </details>
                </div>
            </li>
        </ol>
        <p v-if="matching.length > MAX_ROWS" class="empty">
            Showing the latest {{ MAX_ROWS }} of {{ matching.length }} — download for the rest
        </p>
    </Tile>
</template>

<style scoped>
.toolbar {
    display: flex;
    flex-wrap: wrap;
    justify-content: space-between;
    align-items: center;
    gap: var(--gap-rows);
    margin-bottom: 0.75rem;
}

.chips,
.actions {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: var(--gap-items);
}

.chip,
.action {
    font: inherit;
    font-size: 0.75rem;
    padding: 0.25rem 0.5rem;
    border-radius: var(--radius-badge);
    border: 1px solid var(--border);
    background: transparent;
    color: var(--ink-muted);
    cursor: pointer;
    text-decoration: none;
}

.chip[aria-pressed='true'] {
    background: color-mix(in srgb, var(--series-1) 14%, var(--surface-tile));
    color: var(--ink-primary);
}

.action {
    color: var(--ink-secondary);
}

.routine {
    display: flex;
    align-items: center;
    gap: var(--gap-tight);
    font-size: 0.75rem;
    color: var(--ink-secondary);
}

.empty {
    margin: 0;
    font-size: 0.8125rem;
    color: var(--ink-muted);
}

.list {
    list-style: none;
    margin: 0;
    padding: 0 0.25rem 0 0;
    display: grid;
    row-gap: var(--gap-rows);
    max-height: 28rem;
    overflow-y: auto;
}

.row {
    display: grid;
    grid-template-columns: auto 1fr;
    column-gap: var(--gap-rows);
}

.dot {
    width: 0.5rem;
    height: 0.5rem;
    border-radius: 50%;
    margin-top: 0.3rem;
    background: var(--ink-muted);
}

.row[data-severity='fault'] .dot {
    background: var(--status-critical);
}

.row[data-severity='ok'] .dot {
    background: var(--status-good);
}

.text {
    display: grid;
    gap: 0.0625rem;
    min-width: 0;
}

.line {
    display: flex;
    flex-wrap: wrap;
    align-items: baseline;
    column-gap: var(--gap-rows);
    font-size: 0.8125rem;
}

.time {
    font-variant-numeric: tabular-nums;
    color: var(--ink-muted);
}

.category {
    font-size: 0.7rem;
    font-weight: 600;
    text-transform: uppercase;
    color: var(--ink-muted);
}

.title {
    font-weight: 600;
    color: var(--ink-primary);
}

.row[data-severity='fault'] .title {
    color: var(--status-critical-text);
}

.row[data-severity='info'] .title {
    font-weight: 500;
    color: var(--ink-secondary);
}

.description {
    font-size: 0.75rem;
    color: var(--ink-muted);
}

.details summary {
    font-size: 0.75rem;
    color: var(--ink-secondary);
    cursor: pointer;
    margin: 0.125rem 0 0.375rem;
}
</style>
