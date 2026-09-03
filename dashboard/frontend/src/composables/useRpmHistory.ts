import { onMounted, onUnmounted, reactive } from 'vue'
import { onTelemetryFrame } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/telemetry'

export interface RpmSample {
    /** Seconds since car startup. */
    t: number
    rpm: number
}

const WINDOW_SECONDS = 20 * 60

export function useRpmHistory() {
    const samples = reactive<RpmSample[]>([])

    let lastUptimeMs: number | undefined

    function handleFrame(frame: TelemetryFrame) {
        if (frame.motor_rpm === undefined) return
        if (frame.uptime === undefined) return

        if (lastUptimeMs !== undefined && frame.uptime < lastUptimeMs) {
            // uptime went backwards — the car rebooted. Start a fresh
            // timeline rather than plot overlapping sessions.
            samples.splice(0, samples.length)
        }
        lastUptimeMs = frame.uptime

        const t = frame.uptime / 1000

        samples.push({ t, rpm: frame.motor_rpm })

        const cutoff = t - WINDOW_SECONDS
        while (samples.length > 0 && samples[0]!.t < cutoff) {
            samples.shift()
        }
    }

    let unsubscribe: (() => void) | undefined

    onMounted(() => {
        unsubscribe = onTelemetryFrame(handleFrame)
    })
    onUnmounted(() => {
        unsubscribe?.()
    })

    return { samples }
}
