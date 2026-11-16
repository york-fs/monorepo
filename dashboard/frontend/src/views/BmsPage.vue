<script setup lang="ts">
import { computed, ref } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { useSegments } from '@/composables/useSegments'
import { isFlagOnline } from '@/telemetry'
import { cellVoltageSeverity } from '@/domain/bms'
import { packCellStats, SEGMENT_LETTERS, THERMISTOR_MAPPING_CONFIRMED } from '@/domain/segments'
import { severityOf } from '@/domain/severity'
import { formatVolts } from '@/utils/formatVolts'
import Section from '@/components/Section.vue'
import Tile from '@/components/Tile.vue'
import AutoGrid from '@/components/AutoGrid.vue'
import ExplanationSub from '@/components/ExplanationSub.vue'
import BmsSection from '@/components/bms/BmsSection.vue'
import PackTopView from '@/components/bms/PackTopView.vue'
import SegmentSidesView from '@/components/bms/SegmentSidesView.vue'
import SegmentDetail from '@/components/bms/SegmentDetail.vue'
import SegmentReadingList from '@/components/bms/SegmentReadingList.vue'

const { frame } = useTelemetry()

// Segment A until one is clicked on the pack.
const selected = ref(0)

const online = computed(() => isFlagOnline(frame.online_flags, 'BMS_ONLINE'))

// Which cell is the pack's lowest, since that's the one that ends the run.
const { readings } = useSegments()
const cells = computed(() => packCellStats(readings))
const lowestCellLabel = computed(() => {
    const lowest = cells.value.lowest
    return lowest ? `Segment ${SEGMENT_LETTERS[lowest.segment]}, C${lowest.cell}` : 'No signal'
})
const spread = computed(() => {
    const { lowest, highest } = cells.value
    if (lowest === undefined || highest === undefined) return '—'
    return `${Math.round((highest - lowest.volts) * 1000)} mV`
})
</script>

<template>
    <BmsSection />
    <Section :online="online" title="Segments">
        <AutoGrid min="12rem">
            <Tile
                title="Lowest cell"
                :value="formatVolts(cells.lowest?.volts, 3)"
                :severity="severityOf(cells.lowest?.volts, cellVoltageSeverity)"
            >
                <template #sub>
                    <ExplanationSub :text="lowestCellLabel" />
                </template>
            </Tile>
            <Tile title="Pack spread" :value="spread" />
        </AutoGrid>
        <Tile title="Pack, from above" :level="3">
            <PackTopView v-model="selected" />
        </Tile>
        <SegmentDetail :segment="selected" />
        <Tile :title="`Segment ${SEGMENT_LETTERS[selected]}, both sides`" :level="3">
            <p v-if="!THERMISTOR_MAPPING_CONFIRMED" class="note">
                Thermistor positions are provisional until the mapping to BMS indices is confirmed
                on the car. The readings by index below are always right.
            </p>
            <SegmentSidesView :segment="selected" />
        </Tile>
        <SegmentReadingList :segment="selected" />
    </Section>
</template>

<style scoped>
.note {
    margin: 0 0 0.75rem;
    font-size: 0.8125rem;
    color: var(--status-warning-text);
}
</style>
