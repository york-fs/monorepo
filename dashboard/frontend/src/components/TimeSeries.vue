<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref, useTemplateRef } from 'vue'
import { Line } from 'vue-chartjs'
import type { ChartComponentRef } from 'vue-chartjs'
import { Chart as ChartJS, LinearScale, LineElement, PointElement, Tooltip } from 'chart.js'
import type { Scale, TooltipItem } from 'chart.js'
import zoomPlugin from 'chartjs-plugin-zoom'
import { formatUptime, formatUptimeWithMs } from '@/utils/formatUptime'

// No `Legend`: the legend is rendered as DOM in this component's own header
// row instead (see below), so Chart.js's canvas legend plugin isn't needed.
ChartJS.register(LinearScale, LineElement, PointElement, Tooltip, zoomPlugin)

// Chart.js draws all of its own text onto the canvas — axis ticks, tooltips —
// from a JS-side default stack ('Helvetica Neue', Helvetica, Arial), with no
// way to inherit the page's font. Left alone, every chart's labels are
// visibly a different typeface to everything around them. Read once at
// module load and again on mount, for the same stylesheet-race reason as the
// colours below.
function applyChartFont() {
    const family = readCssVar('--font-sans')
    if (family) ChartJS.defaults.font.family = family
}

applyChartFont()

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
    }>(),
    { showLegend: true },
)

const chartRef = useTemplateRef<ChartComponentRef<'line'>>('chartRef')

function resetZoom() {
    chartRef.value?.chart?.resetZoom()
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
    // properties (the webfont is bundled rather than fetched now, but any
    // future stylesheet-loading hiccup could do the same). Re-read once more
    // after mount as a cheap self-correction; if the first read was already
    // correct this is a no-op.
    handleThemeChange()
    applyChartFont()
})
onUnmounted(() => {
    media?.removeEventListener('change', handleThemeChange)
})

// The canvas itself is opaque to assistive tech. Named from the series
// rather than a new prop — the enclosing SectionPanel's heading already says
// what the chart is *of*, so this only needs to say what's plotted on it.
const chartLabel = computed(() =>
    props.isEmpty
        ? props.emptyMessage
        : `Time series chart of ${props.series.map((line) => line.label).join(', ')}`,
)

const chartData = computed(() => ({
    datasets: props.series.map((s) => ({
        label: s.label,
        data: s.data,
        borderColor: colors.value[s.color],
        backgroundColor: colors.value[s.color],
        borderWidth: 2,
        pointRadius: 0,
        stepped: s.stepped ?? false,
    })),
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
                // Only show millisecond precision once zoomed in far enough for it to
                // matter — otherwise every tick ends in a noisy ".000".
                callback: function (this: Scale, value: number | string) {
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
                // No `stepSize` on purpose: setting one makes Chart.js skip
                // its default `maxTicksLimit` of 11 and generate a tick per
                // step across the whole range (hundreds, for a 0–400 V bus),
                // then auto-skip down to ~11 of them — which lands on
                // arbitrary values like 23/47/71 instead of round numbers.
                precision: 0,
                callback: (value: number | string) => Math.round(Number(value)),
            },
        },
    },
    plugins: {
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
            // Floor the visible span at a second — without it a wheel-zoom
            // runs away into a window narrower than the sample interval,
            // leaving an empty plot with no way back but the reset button.
            // Deliberately no min/max clamp: the plugin captures "original"
            // bounds at chart init, which for a chart that starts empty and
            // grows would then lock panning out of the real data range.
            limits: { x: { minRange: 1 } },
        },
    },
}))
</script>

<template>
    <div class="chart">
        <!-- Legend and reset button both sit in normal flow above the
             canvas. Chart.js's own legend is drawn inside the canvas, which
             put it in a different typeface and left it unable to line up
             with the panel's heading. Keeping both in the DOM also means the
             button no longer overlays the plot area — which is what used to
             need a measured `layout.padding.top` (and a ResizeObserver on
             the button) to stop it clipping into the grid. -->
        <div class="chart-header">
            <ul v-if="showLegend" class="legend">
                <li v-for="line in series" :key="line.label" class="legend-item">
                    <span class="swatch" :class="line.color" />
                    {{ line.label }}
                </li>
            </ul>
            <button type="button" class="reset-zoom" @click="resetZoom">Reset zoom</button>
        </div>

        <div class="chart-wrap" role="img" :aria-label="chartLabel">
            <Line ref="chartRef" :data="chartData" :options="chartOptions" />
            <p v-if="isEmpty" class="empty">{{ emptyMessage }}</p>
        </div>
    </div>
</template>

<style scoped>
.chart {
    display: grid;
    grid-template-rows: auto 1fr;
    gap: 0.5rem;
    min-height: 16.25rem;
}

.chart-header {
    display: flex;
    align-items: center;
    gap: 0.75rem;
}

.legend {
    list-style: none;
    margin: 0;
    padding: 0;
    display: flex;
    flex-wrap: wrap;
    gap: 0.25rem 0.875rem;
}

.legend-item {
    display: inline-flex;
    align-items: center;
    gap: 0.375rem;
    font-size: 0.6875rem;
    color: var(--ink-muted);
}

/* A short rule rather than a dot — it reads as "this is a line on the
   chart", and matches the 2px borderWidth the lines are actually drawn at. */
.swatch {
    width: 0.875rem;
    height: 2px;
    border-radius: 1px;
    flex-shrink: 0;
}
.swatch.series1 {
    background: var(--series-1);
}
.swatch.series2 {
    background: var(--series-2);
}

.reset-zoom {
    /* Never shrink below the label; the legend wraps instead. `auto` keeps
       it hard right whether or not there's a legend to its left. */
    flex-shrink: 0;
    margin-left: auto;
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

.chart-wrap {
    position: relative;
}

/* The canvas must not contribute to .chart-wrap's own size, or the two feed
   back into each other: Chart.js sets explicit pixel dimensions on the
   canvas from the container's current size, so if the container's size is
   itself computed from its content (which includes the canvas), it can
   grow but can never shrink back down. Taking the canvas out of flow breaks
   the loop — chart-wrap's size then comes purely from CSS (the 1fr row
   above, plus grid stretch), and Chart.js's own resize observer keeps the
   canvas matched to it. */
.chart-wrap :deep(canvas) {
    position: absolute;
    inset: 0;
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
