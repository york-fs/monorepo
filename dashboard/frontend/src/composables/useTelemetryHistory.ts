import { markRaw, onMounted, onUnmounted, ref } from 'vue'
import { onTelemetryFrame } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/telemetry'

export interface TimeSeriesPoint {
    /** Seconds since car startup. */
    x: number
    y: number
}

const WINDOW_SECONDS = 20 * 60

// Decoupled from the ~10Hz telemetry frame rate — see `version` below for
// why this needs to be a deliberate throttle rather than "every frame".
const CHART_UPDATE_INTERVAL_MS = 250

/**
 * Generic "buffer telemetry field(s) as {x,y}[] time series, capped at a
 * rolling window, reset on car reboot (uptime decreasing)" composable —
 * extracted once the precharge-voltage/RPM/motor-current histories turned
 * out to be identical apart from which frame field(s) they read. `fields`
 * maps an output key to a selector reading that key's value (or undefined)
 * off a frame; returns one point array per key, keyed the same way.
 *
 * The point arrays are `markRaw` — deliberately kept out of Vue's reactivity
 * entirely, pushed/shifted in place rather than replaced, and handed
 * straight to Chart.js as dataset data (see TimeSeries.vue). Chart.js
 * attaches its own internal (circular) bookkeeping onto whatever data it's
 * given; wrapping that in a deep-reactive Vue proxy — tried first — recursed
 * into that circular graph and blew the stack ("too much recursion").
 * `version` is a plain counter bumped periodically (see below) purely so
 * callers have something reactive to depend on to know new points arrived,
 * without Vue ever touching the arrays/points themselves.
 *
 * `version` is bumped on a `CHART_UPDATE_INTERVAL_MS` timer rather than on
 * every processed frame — data still lands in `points` immediately, this
 * only throttles how often callers are told to redraw. Redrawing (and
 * re-running Chart.js's scale min/max scan) on every ~100ms frame is wasted
 * work once the buffer holds thousands of points; a few redraws a second is
 * plenty for a human to perceive as live. (This used to also be load-bearing
 * for a decimation-shimmer bug — see `TimeSeries.vue` — but decimation's
 * been removed entirely since; kept now purely for the redraw-cost reason.)
 */
export function useTelemetryHistory<K extends string>(
    fields: Record<K, (frame: TelemetryFrame) => number | undefined>,
) {
    const keys = Object.keys(fields) as K[]
    const points = Object.fromEntries(
        keys.map((k) => [k, markRaw<TimeSeriesPoint[]>([])]),
    ) as Record<K, TimeSeriesPoint[]>
    const version = ref(0)

    let lastUptimeMs: number | undefined
    let updateTimer: ReturnType<typeof setTimeout> | undefined

    function handleFrame(frame: TelemetryFrame) {
        if (frame.uptime === undefined) return

        const values = keys.map((k) => [k, fields[k](frame)] as const)
        if (values.every(([, v]) => v === undefined)) return

        const reboot = lastUptimeMs !== undefined && frame.uptime < lastUptimeMs
        lastUptimeMs = frame.uptime

        const t = frame.uptime / 1000

        if (reboot) {
            // uptime went backwards — the car rebooted. Start a fresh
            // timeline rather than plot overlapping sessions.
            for (const k of keys) points[k].splice(0, points[k].length)
        }

        for (const [k, v] of values) {
            if (v !== undefined) points[k].push({ x: t, y: v })
        }

        const cutoff = t - WINDOW_SECONDS
        for (const k of keys) {
            const arr = points[k]
            while (arr.length > 0 && arr[0]!.x < cutoff) arr.shift()
        }

        if (updateTimer === undefined) {
            updateTimer = setTimeout(() => {
                updateTimer = undefined
                version.value++
            }, CHART_UPDATE_INTERVAL_MS)
        }
    }

    let unsubscribe: (() => void) | undefined

    onMounted(() => {
        unsubscribe = onTelemetryFrame(handleFrame)
    })
    onUnmounted(() => {
        unsubscribe?.()
        clearTimeout(updateTimer)
    })

    return { points, version }
}
