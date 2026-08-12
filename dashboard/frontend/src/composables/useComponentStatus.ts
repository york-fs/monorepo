import { computed, toValue, type MaybeRefOrGetter } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import type { TelemetryStatus } from '@/types/telemetry'

/**
 * Combines the overall telemetry link status with a component's own online
 * signal, if it has one — offline if either is offline. `online` undefined
 * (component has no distinguishable signal yet) falls back to the link
 * status alone.
 */
export function useComponentStatus(online?: MaybeRefOrGetter<boolean | undefined>) {
    const { status: linkStatus } = useTelemetry()

    const status = computed<TelemetryStatus>(() => {
        if (linkStatus.value === 'offline') return 'offline'
        if (online !== undefined && toValue(online) === false) return 'offline'
        return 'online'
    })

    return { status }
}
