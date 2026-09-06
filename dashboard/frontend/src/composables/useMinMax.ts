import { onMounted, onUnmounted, ref } from 'vue'
import { onTelemetryFrame } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/telemetry'

/**
 * Tracks the lowest/highest value of a derived reading seen since this
 * composable was mounted (i.e. since the page was opened) — not a history
 * buffer, just two running extrema. `selectMin` reads (or derives) the
 * reading from each frame; return `undefined` for frames with no signal.
 *
 * `selectMax` defaults to `selectMin`, and only differs where the two
 * extrema come from *different* wire fields: LV rail voltage is the one
 * such case, tracking the lowest `lvs_min_voltage` against the highest
 * `lvs_max_voltage` (the backend sends the per-frame min/max across all
 * fuses rather than every per-fuse voltage — see plan/DISTRIBUTION.md).
 */
export function useMinMax(
    selectMin: (frame: TelemetryFrame) => number | undefined,
    selectMax: (frame: TelemetryFrame) => number | undefined = selectMin,
) {
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

        const low = selectMin(frame)
        if (low !== undefined) {
            everMin.value = everMin.value === undefined ? low : Math.min(everMin.value, low)
        }

        const high = selectMax(frame)
        if (high !== undefined) {
            everMax.value = everMax.value === undefined ? high : Math.max(everMax.value, high)
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
