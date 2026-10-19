<script setup lang="ts">
import { computed, ref } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { isFlagOnline } from '@/telemetry'
import { SEGMENT_LETTERS, THERMISTOR_MAPPING_CONFIRMED } from '@/domain/segments'
import Section from '@/components/Section.vue'
import Tile from '@/components/Tile.vue'
import BmsSection from '@/components/bms/BmsSection.vue'
import PackTopView from '@/components/bms/PackTopView.vue'
import SegmentSidesView from '@/components/bms/SegmentSidesView.vue'
import SegmentDetail from '@/components/bms/SegmentDetail.vue'
import SegmentReadingList from '@/components/bms/SegmentReadingList.vue'

const { frame } = useTelemetry()

// Segment A until one is clicked on the pack.
const selected = ref(0)

const online = computed(() => isFlagOnline(frame.online_flags, 'BMS_ONLINE'))
</script>

<template>
    <BmsSection />
    <Section :online="online" title="Segments">
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
