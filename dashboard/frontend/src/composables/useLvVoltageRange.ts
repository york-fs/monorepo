import { onMounted, onUnmounted, ref } from 'vue'
import { onTelemetryFrame } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/types/telemetry'

/**
 * Tracks the lowest/highest `lvs_min_voltage`/`lvs_max_voltage` seen since
 * this composable was mounted (i.e. since the page was opened) — not a
 * history buffer, just two running extrema, since that's all an "ever"
 * range needs.
 */
export function useLvVoltageRange() {
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

        if (frame.lvs_min_voltage !== undefined) {
            everMin.value =
                everMin.value === undefined
                    ? frame.lvs_min_voltage
                    : Math.min(everMin.value, frame.lvs_min_voltage)
        }
        if (frame.lvs_max_voltage !== undefined) {
            everMax.value =
                everMax.value === undefined
                    ? frame.lvs_max_voltage
                    : Math.max(everMax.value, frame.lvs_max_voltage)
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
