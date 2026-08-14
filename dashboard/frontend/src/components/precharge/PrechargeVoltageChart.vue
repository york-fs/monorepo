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
import { useVoltageHistory } from '@/composables/useVoltageHistory'
import { formatUptimeWithMs } from '@/utils/formatUptime'

ChartJS.register(LinearScale, LineElement, PointElement, Legend, Tooltip, Decimation, zoomPlugin)

const { samples } = useVoltageHistory()

const chartRef = useTemplateRef<ChartComponentRef<'line'>>('chartRef')

function resetZoom() {
    chartRef.value?.chart?.resetZoom()
}

// The built-in legend's "start" alignment is relative to the whole canvas
// width, not the plot area — it doesn't know about the space the y-axis
// labels take up to their left. Nudge it over to line up once layout (and
// therefore the y-axis width) is settled.
const alignLegendPlugin = {
    id: 'alignLegendToChartArea',
    afterLayout(chart: ChartJS) {
        if (chart.legend) {
            chart.legend.left = chart.chartArea.left
        }
    },
}

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
})
onUnmounted(() => {
    media?.removeEventListener('change', handleThemeChange)
})

const chartData = computed(() => ({
    datasets: [
        {
            label: 'Precharge rail',
            data: samples
                .filter((s) => s.prchg !== undefined)
                .map((s) => ({ x: s.t, y: s.prchg as number })),
            borderColor: colors.value.series1,
            backgroundColor: colors.value.series1,
            borderWidth: 2,
            pointRadius: 0,
            pointStyle: 'line' as const,
            stepped: true as const,
        },
        {
            label: 'Tractive system',
            data: samples
                .filter((s) => s.ts !== undefined)
                .map((s) => ({ x: s.t, y: s.ts as number })),
            borderColor: colors.value.series2,
            backgroundColor: colors.value.series2,
            borderWidth: 2,
            pointRadius: 0,
            pointStyle: 'line' as const,
            stepped: true as const,
        },
    ],
}))

const chartOptions = computed(() => ({
    responsive: true,
    maintainAspectRatio: false,
    animation: false as const,
    parsing: false as const,
    interaction: { mode: 'nearest' as const, intersect: false, axis: 'x' as const },
    scales: {
        x: {
            type: 'linear' as const,
            grid: { display: false },
            ticks: {
                color: colors.value.text,
                font: { size: 10 },
                callback: (value: number | string) => formatUptimeWithMs(Number(value)),
            },
        },
        y: {
            grid: { color: colors.value.grid },
            ticks: {
                color: colors.value.text,
                font: { size: 10 },
                stepSize: 1,
                precision: 0,
                callback: (value: number | string) => Math.round(Number(value)),
            },
        },
    },
    plugins: {
        legend: {
            position: 'top' as const,
            align: 'start' as const,
            labels: {
                color: colors.value.text,
                usePointStyle: true,
                pointStyleWidth: 16,
                font: { size: 12 },
            },
        },
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
        <button type="button" class="reset-zoom" @click="resetZoom">Reset zoom</button>
        <Line
            ref="chartRef"
            :data="chartData"
            :options="chartOptions"
            :plugins="[alignLegendPlugin]"
        />
        <p v-if="samples.length === 0" class="empty">Waiting for precharge voltage data…</p>
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
