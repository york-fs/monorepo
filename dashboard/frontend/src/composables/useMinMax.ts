import { onMounted, onUnmounted, ref } from 'vue'
import { onTelemetryFrame } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/telemetry'

/**
 * Tracks the lowest/highest value of a single derived reading seen since
 * this composable was mounted (i.e. since the page was opened) — not a
 * history buffer, just two running extrema. `select` reads (or derives) the
 * reading from each frame; return `undefined` for frames with no signal.
 */
export function useMinMax(select: (frame: TelemetryFrame) => number | undefined) {
    const everMin = ref<number>()
    const everMax = ref<number>()

    let lastUptimeMs: number | undefined

    function handleFrame(frame: TelemetryFrame) {
        if (frame.uptime !== undefined) {
            if (lastUptimeMs !== undefined && frame.uptime < lastUptimeMs) {
                // uptime went backwards — the car rebooted. Start a fresh
                // ever-range rather than keep extrema from a prior session.
                everMin.value = undefined
                everMax.value = undefined
            }
            lastUptimeMs = frame.uptime
        }

        const value = select(frame)
        if (value !== undefined) {
            everMin.value = everMin.value === undefined ? value : Math.min(everMin.value, value)
            everMax.value = everMax.value === undefined ? value : Math.max(everMax.value, value)
        }
    }

    let unsubscribe: (() => void) | undefined

    onMounted(() => {
        unsubscribe = onTelemetryFrame(handleFrame)
    })
    onUnmounted(() => {
        unsubscribe?.()
    })

    return { everMin, everMax }
}
