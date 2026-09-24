<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { isFlagOnline } from '@/telemetry'
import type { TelemetryFrame } from '@/telemetry'
import {
    inverterInputVoltageSeverity,
    inverterTemperatureSeverity,
    motorRpmToMph,
    motorTemperatureSeverity,
    rpmSeverity,
} from '@/domain/powertrain'
import { formatPercent } from '@/utils/formatPercent'
import { formatCelsius } from '@/utils/formatCelsius'
import { formatVolts } from '@/utils/formatVolts'
import Section from '@/components/Section.vue'
import Tile from '@/components/Tile.vue'
import AutoGrid from '@/components/AutoGrid.vue'
import MetricTile from '@/components/MetricTile.vue'
import InverterFaultTile from '@/components/powertrain/InverterFaultTile.vue'
import RpmChart from '@/components/powertrain/RpmChart.vue'
import CurrentChart from '@/components/powertrain/CurrentChart.vue'

const { frame } = useTelemetry()

// Formatters and frame selectors for the MetricTiles below. Defined here
// rather than inline in the template so their identity is stable across
// renders — the tiles re-render on every frame anyway, but a stable prop
// keeps the diff honest.
//
// Note the speed tile's underlying quantity is motor RPM, with mph as purely
// a display format of it. That keeps its severity keyed off the real rev
// limit (`rpmSeverity`) rather than off a figure derived through the
// still-placeholder wheel/gearing constants — and since `motorRpmToMph` is
// linear and monotonic, the RPM ever-min/max are the mph ever-min/max too.
const formatRpmAsMph = (rpm: number) => `${Math.round(motorRpmToMph(rpm))} mph`

const selectRpm = (f: TelemetryFrame) => f.motor_rpm
const selectDcVoltage = (f: TelemetryFrame) => f.inverter_input_voltage
const selectInverterTemperature = (f: TelemetryFrame) => f.inverter_temperature
const selectMotorTemperature = (f: TelemetryFrame) => f.motor_temperature

// Depends on both the inverter's own signal and front distribution's — the
// inverter and the pedal (via front distribution) are both part of this
// chain. `false` if either reads offline; `undefined` (no signal yet) only
// once *neither* has reported, so one flag arriving doesn't get masked by
// the other still being unknown. See plan/POWERTRAIN.md.
const online = computed<boolean | undefined>(() => {
    const inverter = isFlagOnline(frame.online_flags, 'INVERTER_ONLINE')
    const front = isFlagOnline(frame.online_flags, 'FRONT_ONLINE')
    if (inverter === false || front === false) return false
    if (inverter === undefined && front === undefined) return undefined
    return true
})
</script>

<template>
    <Section :online="online" title="Powertrain">
        <div class="summary">
            <AutoGrid>
                <MetricTile
                    name="Speed"
                    :value="frame.motor_rpm"
                    :format="formatRpmAsMph"
                    :select="selectRpm"
                    :severity-of="rpmSeverity"
                />
                <InverterFaultTile :fault="frame.inverter_fault" />
            </AutoGrid>
            <AutoGrid>
                <MetricTile
                    name="DC input voltage"
                    :value="frame.inverter_input_voltage"
                    :format="formatVolts"
                    :select="selectDcVoltage"
                    :severity-of="inverterInputVoltageSeverity"
                />
                <MetricTile
                    name="Inverter temperature"
                    :value="frame.inverter_temperature"
                    :format="formatCelsius"
                    :select="selectInverterTemperature"
                    :severity-of="inverterTemperatureSeverity"
                />
                <MetricTile
                    name="Motor temperature"
                    :value="frame.motor_temperature"
                    :format="formatCelsius"
                    :select="selectMotorTemperature"
                    :severity-of="motorTemperatureSeverity"
                />
            </AutoGrid>
        </div>

        <div class="charts">
            <Tile title="RPM history" chart>
                <RpmChart />
            </Tile>
            <Tile title="Current history" chart>
                <CurrentChart />
            </Tile>
        </div>

        <div>
            <h3>APPS</h3>
            <AutoGrid>
                <!-- No ever-range or severity yet: APPS grows its own error
                flags/states next, which is what will decide what a "bad"
                pedal reading looks like. -->
                <MetricTile
                    name="Pedal travel"
                    :value="frame.pedal_travel"
                    :format="formatPercent"
                />
            </AutoGrid>
        </div>
    </Section>
</template>

<style scoped>
.summary {
    display: grid;
    gap: var(--gap-tiles);
}

.charts {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: var(--gap-tiles);
}

@media (max-width: 47.5em) {
    .charts {
        grid-template-columns: 1fr;
    }
}
</style>
