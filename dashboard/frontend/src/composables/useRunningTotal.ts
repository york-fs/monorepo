import { onMounted, onUnmounted, ref } from 'vue'
import { onTelemetryFrame } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/telemetry'

// Longest gap between frames to integrate across. Anything longer is a
// dropped link, and assuming the car held its rate through it would guess.
const MAX_STEP_S = 0.5

/**
 * A rate from each frame (per second), totalled since the first frame that
 * carried it or the last reset, along with when that was. Unlike the
 * ever-ranges it carries on across a car reboot: a reboot doesn't undo the
 * distance covered or the energy used.
 */
export function useRunningTotal(rateOf: (frame: TelemetryFrame) => number | undefined) {
    const total = ref(0)
    const since = ref<Date>()

    let last: { uptime: number; rate: number } | undefined

    function handleFrame(frame: TelemetryFrame) {
        const rate = rateOf(frame)
        if (frame.uptime === undefined || rate === undefined) return
        since.value ??= new Date()
        if (last) {
            // Negative across a reboot, when uptime starts again from zero.
            const step = (frame.uptime - last.uptime) / 1000
            if (step > 0 && step <= MAX_STEP_S) total.value += ((last.rate + rate) / 2) * step
        }
        last = { uptime: frame.uptime, rate }
    }

    let unsubscribe: (() => void) | undefined

    onMounted(() => {
        unsubscribe = onTelemetryFrame(handleFrame)
    })
    onUnmounted(() => {
        unsubscribe?.()
    })

    function reset() {
        total.value = 0
        since.value = undefined
        last = undefined
    }

    return { total, since, reset }
}
