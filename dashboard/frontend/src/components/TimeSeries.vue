<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref, useTemplateRef } from 'vue'
import { Line } from 'vue-chartjs'
import type { ChartComponentRef } from 'vue-chartjs'
import {
    Chart as ChartJS,
    LinearScale,
    LineElement,
    PointElement,
    Legend,
    Tooltip,
    Decimation,
} from 'chart.js'
import type { TooltipItem } from 'chart.js'
import zoomPlugin from 'chartjs-plugin-zoom'
import { formatUptime, formatUptimeWithMs } from '@/utils/formatUptime'

ChartJS.register(LinearScale, LineElement, PointElement, Legend, Tooltip, Decimation, zoomPlugin)

export interface TimeSeriesLine {
    label: string
    /** Which theme colour slot (`--series-1`/`--series-2`) this line uses. */
    color: 'series1' | 'series2'
    data: { x: number; y: number }[]
    /** e.g. precharge's rail/TS-bus traces, which hold between samples. */
    stepped?: boolean
}

const props = withDefaults(
    defineProps<{
        series: TimeSeriesLine[]
        /** No dataset has any samples yet — shows `emptyMessage` instead. */
        isEmpty: boolean
        emptyMessage: string
        /** Hide for a single-line chart, where a legend has nothing to add. */
        showLegend?: boolean
        /** Forces integer-spaced y ticks (e.g. precharge's whole-volt steps). */
        yStepSize?: number
    }>(),
    { showLegend: true },
)

const chartRef = useTemplateRef<ChartComponentRef<'line'>>('chartRef')
const resetZoomRef = useTemplateRef<HTMLButtonElement>('resetZoomRef')

function resetZoom() {
    chartRef.value?.chart?.resetZoom()
}

// Without a legend there's nothing pushing the plot area down, so the
// top-right reset-zoom button (positioned over the canvas) would clip into
// the grid — reserve its actual rendered height (plus a small gap) as chart
// padding instead, rather than guessing a fixed number that can drift out of
// sync with the button's real CSS.
const resetZoomHeight = ref(0)
let resetZoomObserver: ResizeObserver | undefined

function readCssVar(name: string): string {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim()
}

function readThemeColors() {
    return {
        series1: readCssVar('--series-1'),
        series2: readCssVar('--series-2'),
        grid: readCssVar('--border'),
        text: readCssVar('--ink-muted'),
    }
}

const colors = ref(readThemeColors())

// Chart.js colors are baked into the config, not CSS — re-read them if the
// viewer's OS theme flips (this app has no in-app theme toggle yet).
let media: MediaQueryList | undefined
function handleThemeChange() {
    colors.value = readThemeColors()
}

onMounted(() => {
    media = window.matchMedia('(prefers-color-scheme: dark)')
    media.addEventListener('change', handleThemeChange)

    // The initial read above can race the stylesheet defining these custom
    // properties (e.g. a slow/blocked webfont @import used to delay it —
    // removed now, but any future stylesheet-loading hiccup could do the
    // same). Re-read once more after mount as a cheap self-correction; if
    // the first read was already correct this is a no-op.
    handleThemeChange()

    if (resetZoomRef.value) {
        resetZoomObserver = new ResizeObserver(([entry]) => {
            resetZoomHeight.value = entry?.target.getBoundingClientRect().height ?? 0
        })
        resetZoomObserver.observe(resetZoomRef.value)
    }
})
onUnmounted(() => {
    media?.removeEventListener('change', handleThemeChange)
    resetZoomObserver?.disconnect()
})

const chartData = computed(() => ({
    datasets: props.series.map((s) => ({
        label: s.label,
        data: s.data,
        borderColor: colors.value[s.color],
        backgroundColor: colors.value[s.color],
        borderWidth: 2,
        pointRadius: 0,
        pointStyle: 'line' as const,
        stepped: s.stepped ?? false,
    })),
}))

const chartOptions = computed(() => ({
    responsive: true,
    maintainAspectRatio: false,
    animation: false as const,
    parsing: false as const,
    interaction: { mode: 'nearest' as const, intersect: false, axis: 'x' as const },
    layout: { padding: { top: props.showLegend ? 0 : resetZoomHeight.value + 8 } },
    scales: {
        x: {
            type: 'linear' as const,
            grid: { display: false },
            ticks: {
                color: colors.value.text,
                font: { size: 10 },
                // Only show millisecond precision once zoomed in far enough for it to
                // matter — otherwise every tick ends in a noisy ".000".
                callback: function (this: { max: number; min: number }, value: number | string) {
                    const value_ = Number(value)
                    return this.max - this.min < 5
                        ? formatUptimeWithMs(value_)
                        : formatUptime(value_)
                },
            },
        },
        y: {
            grid: { color: colors.value.grid },
            ticks: {
                color: colors.value.text,
                font: { size: 10 },
                stepSize: props.yStepSize,
                precision: 0,
                callback: (value: number | string) => Math.round(Number(value)),
            },
        },
    },
    plugins: {
        legend: props.showLegend
            ? {
                  position: 'top' as const,
                  align: 'start' as const,
                  labels: {
                      color: colors.value.text,
                      usePointStyle: true,
                      pointStyleWidth: 16,
                      font: { size: 12 },
                  },
              }
            : { display: false },
        tooltip: {
            mode: 'nearest' as const,
            intersect: false,
            callbacks: {
                title: (items: TooltipItem<'line'>[]) => {
                    const x = items[0]?.parsed.x
                    return x === undefined || x === null ? '' : formatUptimeWithMs(x)
                },
            },
        },
        decimation: {
            enabled: true,
            algorithm: 'lttb' as const,
            samples: 400,
        },
        zoom: {
            pan: {
                enabled: true,
                mode: 'x' as const,
            },
            zoom: {
                wheel: { enabled: true },
                pinch: { enabled: true },
                mode: 'x' as const,
            },
        },
    },
}))
</script>

<template>
    <div class="chart-wrap">
        <button ref="resetZoomRef" type="button" class="reset-zoom" @click="resetZoom">
            Reset zoom
        </button>
        <Line ref="chartRef" :data="chartData" :options="chartOptions" />
        <p v-if="isEmpty" class="empty">{{ emptyMessage }}</p>
    </div>
</template>

<style scoped>
.chart-wrap {
    min-height: 16.25rem;
    position: relative;
}

/* The canvas must not contribute to .chart-wrap's own size, or the two feed
   back into each other: Chart.js sets explicit pixel dimensions on the
   canvas from the container's current size, so if the container's size is
   itself computed from its content (which includes the canvas), it can
   grow but can never shrink back down. Taking the canvas out of flow breaks
   the loop — chart-wrap's size then comes purely from CSS (min-height plus
   grid stretch), and Chart.js's own resize observer keeps the canvas
   matched to it. */
.chart-wrap :deep(canvas) {
    position: absolute;
    inset: 0;
}

.reset-zoom {
    position: absolute;
    top: 0;
    right: 0;
    z-index: 1;
    background: none;
    border: 1px solid var(--border);
    border-radius: 0.25rem;
    padding: 0.1875rem 0.5rem;
    font-size: 0.6875rem;
    color: var(--ink-muted);
    cursor: pointer;
}

.reset-zoom:hover {
    color: var(--ink-primary);
    border-color: var(--ink-muted);
}

.empty {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    margin: 0;
    font-size: 0.75rem;
    color: var(--ink-muted);
}
</style>
