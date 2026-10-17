import { computed, effectScope, readonly, ref, shallowRef, watch } from 'vue'
import { onLogEvent, useTelemetry } from '@/composables/useTelemetry'
import { isDemoMode } from '@/demo'
import type { LogEvent } from '@/domain/events'

// Module-level singleton, like useTelemetry: one copy of the history shared by
// every component that shows part of it.
const events = shallowRef<LogEvent[]>([])
const available = ref(!isDemoMode())
const loadError = ref<string | null>(null)
let started = false

/**
 * Merges by id rather than appending blindly: a history fetch can overlap
 * with events that already arrived live over the stream, and a reconnect
 * re-fetches everything.
 */
function merge(incoming: LogEvent[]) {
    const byId = new Map(events.value.map((event) => [event.id, event]))
    for (const event of incoming) byId.set(event.id, event)
    events.value = [...byId.values()].sort((a, b) => a.id - b.id)
}

async function fetchHistory() {
    try {
        const response = await fetch('/api/events')
        if (!response.ok) throw new Error(`HTTP ${response.status}`)
        const history = (await response.json()) as LogEvent[]
        // Replace rather than merge, so a clear from another browser is picked
        // up on reconnect — but keep anything that arrived live while the
        // request was in flight. Ids keep counting up across clears.
        const newestId = history.at(-1)?.id ?? 0
        const live = events.value.filter((event) => event.id > newestId)
        events.value = [...history, ...live]
        loadError.value = null
    } catch (error) {
        loadError.value = `Couldn't load the event log (${String(error)})`
    }
}

async function clearHistory() {
    const response = await fetch('/api/events/clear', { method: 'POST' })
    if (response.ok) events.value = []
}

function start() {
    if (started || !available.value) return
    started = true

    onLogEvent((event) => merge([event]))

    // Re-fetch whenever the stream (re)opens: events logged while this
    // browser was disconnected are only available from the history endpoint.
    // Detached so the watcher outlives whichever component happened to call
    // this first.
    effectScope(true).run(() => {
        const { link } = useTelemetry()
        watch(
            link,
            (state) => {
                if (state === 'open') void fetchHistory()
            },
            { immediate: true },
        )
    })
}

export function useEventLog() {
    start()
    return {
        // A computed rather than `readonly()`, which would deep-readonly every
        // event's type for no benefit — the array is replaced, never mutated.
        events: computed<readonly LogEvent[]>(() => events.value),
        available: readonly(available),
        loadError: readonly(loadError),
        clearHistory,
    }
}
