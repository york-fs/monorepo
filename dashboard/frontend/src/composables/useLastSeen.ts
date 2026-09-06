import { computed, onMounted, onUnmounted, ref, type MaybeRefOrGetter } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { useComponentStatus } from '@/composables/useComponentStatus'
import { formatRelativeTime } from '@/utils/formatRelativeTime'

// One shared ticker for all consumers, ref-counted, rather than an interval
// each: every status-bar tile mounts this composable but only the
// rear-distribution one actually renders a duration (see
// ConnectionStatusTile's `hasOwnSignal`), so per-instance timers meant five
// intervals doing one tile's work.
const now = ref(Date.now())
let tickTimer: ReturnType<typeof setInterval> | undefined
let consumers = 0

function retainTicker() {
    consumers += 1
    if (tickTimer !== undefined) return
    now.value = Date.now()
    tickTimer = setInterval(() => {
        now.value = Date.now()
    }, 1000)
}

function releaseTicker() {
    consumers -= 1
    if (consumers > 0) return
    clearInterval(tickTimer)
    tickTimer = undefined
}

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

    onMounted(retainTicker)
    onUnmounted(releaseTicker)

    const relativeText = computed(() => {
        if (lastUpdated.value === null) return '—'
        return formatRelativeTime((now.value - lastUpdated.value) / 1000)
    })

    return { status, relativeText }
}
