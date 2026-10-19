<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import type { TelemetryFrame } from '@/telemetry'
import { cellTemperatureSeverity, cellVoltageSeverity } from '@/domain/bms'
import { formatCelsius } from '@/utils/formatCelsius'
import { formatVolts } from '@/utils/formatVolts'
import Section from '@/components/Section.vue'
import AutoGrid from '@/components/AutoGrid.vue'
import MetricTile from '@/components/MetricTile.vue'
import StatusBar from '@/components/status-bar/StatusBar.vue'
import DistributionSection from '@/components/distribution/DistributionSection.vue'
import InverterFaultTile from '@/components/powertrain/InverterFaultTile.vue'
import BmsErrorList from '@/components/bms/BmsErrorList.vue'

// What to check first when the dash boots: is every board talking, is
// shutdown closed, and is anything already faulted. The full detail for each
// lives on its own page.
const { frame } = useTelemetry()

const formatCellVoltage = (volts: number) => formatVolts(volts, 4)
const selectMinCellVoltage = (f: TelemetryFrame) => f.min_cell_voltage
const selectMaxCellTemperature = (f: TelemetryFrame) => f.max_cell_temperature
</script>

<template>
    <StatusBar />
    <DistributionSection />
    <Section title="Faults">
        <AutoGrid min="12rem">
            <MetricTile
                name="Min cell voltage"
                :value="frame.min_cell_voltage"
                :format="formatCellVoltage"
                :select="selectMinCellVoltage"
                :severity-of="cellVoltageSeverity"
            />
            <MetricTile
                name="Max cell temperature"
                :value="frame.max_cell_temperature"
                :format="formatCelsius"
                :select="selectMaxCellTemperature"
                :severity-of="cellTemperatureSeverity"
            />
            <InverterFaultTile :fault="frame.inverter_fault" />
        </AutoGrid>
        <BmsErrorList :flags="frame.bms_master_error_flags" />
    </Section>
</template>
