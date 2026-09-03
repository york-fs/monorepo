import { onMounted, onUnmounted, reactive } from 'vue'
import { onTelemetryFrame } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/telemetry'

export interface CurrentSample {
    /** Seconds since car startup. */
    t: number
    apps: number | undefined
    motor: number | undefined
}

const WINDOW_SECONDS = 20 * 60

/** Buffers desired (`desired_motor_current`) vs. actual (`motor_current`) AC current. */
export function useCurrentHistory() {
    const samples = reactive<CurrentSample[]>([])

    let lastUptimeMs: number | undefined

    function handleFrame(frame: TelemetryFrame) {
        if (frame.desired_motor_current === undefined && frame.motor_current === undefined) return
        if (frame.uptime === undefined) return

        if (lastUptimeMs !== undefined && frame.uptime < lastUptimeMs) {
            // uptime went backwards — the car rebooted. Start a fresh
            // timeline rather than plot overlapping sessions.
            samples.splice(0, samples.length)
        }
        lastUptimeMs = frame.uptime

        const t = frame.uptime / 1000

        samples.push({ t, apps: frame.desired_motor_current, motor: frame.motor_current })

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
