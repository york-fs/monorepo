import { onMounted, onUnmounted, reactive } from 'vue'
import { onTelemetryFrame } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/telemetry'

export interface VoltageSample {
    /** Seconds since car startup. */
    t: number
    prchg: number | undefined
    ts: number | undefined
}

const WINDOW_SECONDS = 20 * 60

export function useVoltageHistory() {
    const samples = reactive<VoltageSample[]>([])

    let lastUptimeMs: number | undefined

    function handleFrame(frame: TelemetryFrame) {
        if (
            frame.precharge_prchg_voltage === undefined &&
            frame.precharge_ts_voltage === undefined
        ) {
            return
        }
        if (frame.uptime === undefined) return

        if (lastUptimeMs !== undefined && frame.uptime < lastUptimeMs) {
            // uptime went backwards — the car rebooted. Start a fresh
            // timeline rather than plot overlapping sessions.
            samples.splice(0, samples.length)
        }
        lastUptimeMs = frame.uptime

        const t = frame.uptime / 1000

        samples.push({
            t,
            prchg: frame.precharge_prchg_voltage,
            ts: frame.precharge_ts_voltage,
        })

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
