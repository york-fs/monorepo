import { computed, reactive, readonly, ref } from 'vue'
import type { LinkState, TelemetryFrame, TelemetryStatus } from '@/telemetry'
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
}

connect()

/**
 * Subscribe to every raw frame as it arrives, with the timestamp it was
 * received at. Intended for a future history/recording layer (e.g. for
 * charts) to hook into without changing this file or its consumers.
 * Returns an unsubscribe function.
 */
export function onTelemetryFrame(listener: FrameListener) {
    listeners.add(listener)
    return () => listeners.delete(listener)
}

export function useTelemetry() {
    const status = computed<TelemetryStatus>(() => (stale.value ? 'offline' : 'online'))

    return {
        frame: readonly(frame),
        link: readonly(link),
        status,
        lastUpdated: readonly(lastUpdated),
    }
}
