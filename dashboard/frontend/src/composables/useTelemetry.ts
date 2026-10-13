import { computed, reactive, readonly, ref } from 'vue'
import type { LinkState, TelemetryFrame, TelemetryStatus } from '@/telemetry'
import type { LogEvent } from '@/domain/events'
import { isDemoMode, startDemoTelemetry } from '@/demo'

// No frame in this long means treat the data as stale even though the
// SSE socket may still be open — the radio link can go quiet without the
// browser connection closing.
const STALE_AFTER_MS = 3000

const frame = reactive<TelemetryFrame>({})
const link = ref<LinkState>('connecting')
const lastUpdated = ref<number | null>(null)
const stale = ref(true)

type FrameListener = (frame: TelemetryFrame, receivedAt: number) => void
const listeners = new Set<FrameListener>()

type LogEventListener = (event: LogEvent) => void
const logListeners = new Set<LogEventListener>()

let source: EventSource | null = null
let staleTimer: ReturnType<typeof setTimeout> | undefined
let demoStarted = false

function resetStaleTimer() {
    stale.value = false
    clearTimeout(staleTimer)
    staleTimer = setTimeout(() => {
        stale.value = true
    }, STALE_AFTER_MS)
}

function applyFrame(parsed: TelemetryFrame) {
    const receivedAt = Date.now()

    Object.assign(frame, parsed)
    lastUpdated.value = receivedAt
    resetStaleTimer()

    for (const listener of listeners) listener(parsed, receivedAt)
}

function connect() {
    if (isDemoMode()) {
        if (demoStarted) return
        demoStarted = true
        link.value = 'open'
        // The returned stopper is deliberately dropped: this module is a
        // process-lifetime singleton with nothing to tear it down, same as
        // the EventSource below.
        startDemoTelemetry(applyFrame)
        return
    }

    if (source) return

    source = new EventSource('/api/stream')

    source.onopen = () => {
        link.value = 'open'
    }

    source.onerror = () => {
        link.value = source?.readyState === EventSource.CLOSED ? 'closed' : 'connecting'
    }

    source.onmessage = (event) => {
        applyFrame(JSON.parse(event.data) as TelemetryFrame)
    }

    // Event log entries share the stream as a named SSE event so they arrive
    // in order with the frames that caused them.
    source.addEventListener('log', (event) => {
        const parsed = JSON.parse(event.data) as LogEvent
        for (const listener of logListeners) listener(parsed)
    })
}

const status = computed<TelemetryStatus>(() => (stale.value ? 'offline' : 'online'))

/**
 * Subscribe to every raw frame as it arrives, with the timestamp it was
 * received at. Used by the history/extrema composables
 * (`useTelemetryHistory`, `useMinMax`) to buffer frames without going
 * through the `frame` ref. Returns an unsubscribe function.
 */
export function onTelemetryFrame(listener: FrameListener) {
    connect()
    listeners.add(listener)
    return () => listeners.delete(listener)
}

/**
 * Subscribe to event log entries pushed live by the backend (see
 * `useEventLog`). Never fires in demo mode, which has no backend. Returns an
 * unsubscribe function.
 */
export function onLogEvent(listener: LogEventListener) {
    connect()
    logListeners.add(listener)
    return () => logListeners.delete(listener)
}

export function useTelemetry() {
    // Connecting on first use rather than on import: a module-level call
    // would open the stream (or start the demo timers) merely because
    // something transitively imported this file, before the app has mounted.
    // `connect()` is idempotent.
    connect()

    return {
        frame: readonly(frame),
        link: readonly(link),
        status,
        lastUpdated: readonly(lastUpdated),
    }
}
