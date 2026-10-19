<script setup lang="ts">
import { computed } from 'vue'
import { useSegments } from '@/composables/useSegments'
import { cellTemperatureSeverity, cellVoltageSeverity } from '@/domain/bms'
import { severityOf } from '@/domain/severity'
import {
    SEGMENT_ERROR_META,
    SEGMENT_LETTERS,
    segmentErrorSeverity,
    segmentStats,
} from '@/domain/segments'
import { formatCelsius } from '@/utils/formatCelsius'
import { formatVolts } from '@/utils/formatVolts'
import AutoGrid from '@/components/AutoGrid.vue'
import Tile from '@/components/Tile.vue'
import FlagsList, { type FlagRow } from '@/components/FlagsList.vue'

const props = defineProps<{ segment: number }>()

const { readings, isStale, errorFlags } = useSegments()

const letter = computed(() => SEGMENT_LETTERS[props.segment])
const segment = computed(() => readings.get(props.segment))
const stale = computed(() => isStale(props.segment))
const flags = computed(() => errorFlags.value?.[props.segment])
const stats = computed(() => segmentStats(segment.value))

const errorRows = computed<FlagRow[]>(() =>
    (flags.value ?? []).map((flag) => ({
        key: flag,
        label: SEGMENT_ERROR_META[flag].label,
        description: SEGMENT_ERROR_META[flag].description,
        tone: 'fault',
    })),
)

const spread = computed(() => {
    const { minCellVoltage, maxCellVoltage } = stats.value
    if (minCellVoltage === undefined || maxCellVoltage === undefined) return '—'
    return `${Math.round((maxCellVoltage - minCellVoltage) * 1000)} mV`
})
const balancing = computed(() => {
    const cells = segment.value?.balancing_cells
    if (cells === undefined) return '—'
    return cells.length === 0 ? 'None' : cells.map((cell) => `C${cell}`).join(', ')
})
</script>

<template>
    <div class="detail">
        <Tile
            :title="`Segment ${letter} errors`"
            accent
            :severity="segmentErrorSeverity(flags)"
            :level="4"
        >
            <FlagsList
                :rows="errorRows"
                :empty-text="flags === undefined ? 'No signal' : 'No errors'"
            />
        </Tile>

        <AutoGrid min="9rem" :class="{ stale }">
            <Tile
                title="Min cell"
                :value="formatVolts(stats.minCellVoltage, 3)"
                :severity="severityOf(stats.minCellVoltage, cellVoltageSeverity)"
            />
            <Tile
                title="Max cell"
                :value="formatVolts(stats.maxCellVoltage, 3)"
                :severity="severityOf(stats.maxCellVoltage, cellVoltageSeverity)"
            />
            <Tile title="Spread" :value="spread" />
            <Tile
                title="Max temperature"
                :value="formatCelsius(stats.maxTemperature)"
                :severity="severityOf(stats.maxTemperature, cellTemperatureSeverity)"
            />
            <Tile title="Balancing" :value="balancing" />
            <Tile
                title="Max correction"
                :value="segment === undefined ? '—' : String(segment.max_correction)"
            />
        </AutoGrid>
    </div>
</template>

<style scoped>
.detail {
    display: grid;
    gap: var(--gap-tiles);
}

.stale {
    opacity: 0.4;
}
</style>
