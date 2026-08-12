import { computed, onMounted, onUnmounted, ref, type MaybeRefOrGetter } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { useComponentStatus } from '@/composables/useComponentStatus'
import { formatRelativeTime } from '@/utils/formatRelativeTime'

/**
 * Ticks once a second so a "how long ago" readout keeps counting up even
 * though nothing else causes a re-render while offline (no new frames are
 * arriving to trigger reactivity).
 *
 * `online` is an optional per-component signal (see `useComponentStatus`) —
 * omit it to report on the overall link alone.
 */
export function useLastSeen(online?: MaybeRefOrGetter<boolean | undefined>) {
    const { lastUpdated } = useTelemetry()
    const { status } = useComponentStatus(online)

    const now = ref(Date.now())
    let tickTimer: ReturnType<typeof setInterval> | undefined
    onMounted(() => {
        tickTimer = setInterval(() => {
            now.value = Date.now()
        }, 1000)
    })
    onUnmounted(() => clearInterval(tickTimer))

    const relativeText = computed(() => {
        if (lastUpdated.value === null) return '—'
        return formatRelativeTime((now.value - lastUpdated.value) / 1000)
    })

    return { status, relativeText }
}
