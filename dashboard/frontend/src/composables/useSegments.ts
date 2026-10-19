import { computed, ref, shallowReactive } from 'vue'
import { onTelemetryFrame, useTelemetry } from '@/composables/useTelemetry'
import type { SegmentReadings } from '@/telemetry'

// Each segment comes round about every 0.6 s, so one unheard from for this
// long has dropped out and its readings are only its last word.
const STALE_AFTER_MS = 3000

// Module-level, like useEventLog: each frame carries only one segment's
// readings, so the latest of every segment has to be kept somewhere that
// outlives whichever component asked first.
const latest = shallowReactive(new Map<number, SegmentReadings>())
const heardAt = shallowReactive(new Map<number, number>())
// Measured against the last frame rather than a clock, since a dead link
// already dims the whole section.
const lastFrameAt = ref(0)
let started = false

function start() {
    if (started) return
    started = true
    onTelemetryFrame((frame) => {
        lastFrameAt.value = Date.now()
        const { segment } = frame
        // The rear only sends a disconnected segment when none are connected,
        // and its readings are whatever it last had.
        if (!segment || frame.segment_error_flags?.[segment.id]?.includes('DISCONNECTED')) return
        latest.set(segment.id, segment)
        heardAt.set(segment.id, lastFrameAt.value)
    })
}

/**
 * The latest full readings of every BMS segment heard from so far, by segment
 * id, whether each is stale, plus every segment's error flags from the latest
 * frame.
 */
export function useSegments() {
    start()
    const { frame } = useTelemetry()
    return {
        readings: latest as ReadonlyMap<number, SegmentReadings>,
        /** Heard from before, but not lately. */
        isStale: (id: number) => {
            const at = heardAt.get(id)
            return at !== undefined && lastFrameAt.value - at > STALE_AFTER_MS
        },
        errorFlags: computed(() => frame.segment_error_flags),
    }
}
