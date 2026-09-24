<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { isFlagOnline } from '@/telemetry'
import type { TelemetryFrame } from '@/telemetry'
import {
    cellTemperatureSeverity,
    cellVoltageSeverity,
    i2cErrorCountSeverity,
    packCurrentSeverity,
} from '@/domain/bms'
import { formatAmps } from '@/utils/formatAmps'
import { formatCelsius } from '@/utils/formatCelsius'
import { formatVolts } from '@/utils/formatVolts'
import Section from '@/components/Section.vue'
import Tile from '@/components/Tile.vue'
import AutoGrid from '@/components/AutoGrid.vue'
import MetricTile from '@/components/MetricTile.vue'
import BmsCurrentChart from '@/components/bms/BmsCurrentChart.vue'
import BmsErrorList from '@/components/bms/BmsErrorList.vue'

const { frame } = useTelemetry()

// Formatters and frame selectors, declared here so their identity is stable
// across the ~10Hz re-renders — same reason as PowertrainSection's.
// Cell voltages come off the wire at four decimal places, unlike the LV
// rail's whole volts — the interesting movement is in the millivolts.
const formatCellVoltage = (volts: number) => formatVolts(volts, 4)
const formatCount = (count: number) => String(count)

const selectPositiveCurrent = (f: TelemetryFrame) => f.positive_current
const selectNegativeCurrent = (f: TelemetryFrame) => f.negative_current
const selectMinCellVoltage = (f: TelemetryFrame) => f.min_cell_voltage
const selectMaxCellVoltage = (f: TelemetryFrame) => f.max_cell_voltage
const selectMinCellTemperature = (f: TelemetryFrame) => f.min_cell_temperature
const selectMaxCellTemperature = (f: TelemetryFrame) => f.max_cell_temperature

const online = computed(() => isFlagOnline(frame.online_flags, 'BMS_ONLINE'))
</script>

<template>
    <Section :online="online" title="BMS">
        <BmsErrorList :flags="frame.bms_master_error_flags" />

        <div class="summary">
            <AutoGrid min="12rem">
                <MetricTile
                    name="Positive pole current"
                    :value="frame.positive_current"
                    :format="formatAmps"
                    :select="selectPositiveCurrent"
                    :severity-of="packCurrentSeverity"
                />
                <MetricTile
                    name="Negative pole current"
                    :value="frame.negative_current"
                    :format="formatAmps"
                    :select="selectNegativeCurrent"
                    :severity-of="packCurrentSeverity"
                />
                <!-- No ever-range: the count only ever climbs, so its
                     ever-max is just the live value again. -->
                <MetricTile
                    name="i2c errors"
                    :value="frame.bms_i2c_error_count"
                    :format="formatCount"
                    :severity-of="i2cErrorCountSeverity"
                />
            </AutoGrid>
            <AutoGrid min="12rem">
                <MetricTile
                    name="Min cell voltage"
                    :value="frame.min_cell_voltage"
                    :format="formatCellVoltage"
                    :select="selectMinCellVoltage"
                    :severity-of="cellVoltageSeverity"
                />
                <MetricTile
                    name="Max cell voltage"
                    :value="frame.max_cell_voltage"
                    :format="formatCellVoltage"
                    :select="selectMaxCellVoltage"
                    :severity-of="cellVoltageSeverity"
                />
                <MetricTile
                    name="Min cell temperature"
                    :value="frame.min_cell_temperature"
                    :format="formatCelsius"
                    :select="selectMinCellTemperature"
                    :severity-of="cellTemperatureSeverity"
                />
                <MetricTile
                    name="Max cell temperature"
                    :value="frame.max_cell_temperature"
                    :format="formatCelsius"
                    :select="selectMaxCellTemperature"
                    :severity-of="cellTemperatureSeverity"
                />
            </AutoGrid>
        </div>

        <Tile title="Pack current history" chart>
            <BmsCurrentChart />
        </Tile>
    </Section>
</template>

<style scoped>
.summary {
    display: grid;
    gap: var(--gap-tiles);
}
</style>
