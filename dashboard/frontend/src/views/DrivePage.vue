<script setup lang="ts">
import { computed, ref, watchEffect } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { useRunningTotal } from '@/composables/useRunningTotal'
import { isFlagOnline } from '@/telemetry'
import type { TelemetryFrame } from '@/telemetry'
import {
    inverterInputVoltageSeverity,
    inverterTemperatureSeverity,
    METRES_PER_MOTOR_REV,
    MOTOR_CURRENT_MAX,
    motorRpmToMph,
    motorTemperatureSeverity,
    rpmSeverity,
} from '@/domain/powertrain'
import {
    cellTemperatureSeverity,
    cellVoltageSeverity,
    estimateStateOfCharge,
    packCurrentSeverity,
    STATE_OF_CHARGE_MAX_CURRENT,
    stateOfChargeSeverity,
} from '@/domain/bms'
import { lvVoltageSeverity } from '@/domain/lvs'
import { formatAmps } from '@/utils/formatAmps'
import { formatCelsius } from '@/utils/formatCelsius'
import { formatPercent } from '@/utils/formatPercent'
import { formatVolts } from '@/utils/formatVolts'
import Section from '@/components/Section.vue'
import Tile from '@/components/Tile.vue'
import AutoGrid from '@/components/AutoGrid.vue'
import MetricTile from '@/components/MetricTile.vue'
import ExplanationSub from '@/components/ExplanationSub.vue'
import BarTile from '@/components/drive/BarTile.vue'
import TotalTile from '@/components/drive/TotalTile.vue'
import PumpIndicator from '@/components/drive/PumpIndicator.vue'
import InverterFaultTile from '@/components/powertrain/InverterFaultTile.vue'
import RpmChart from '@/components/powertrain/RpmChart.vue'
import CurrentChart from '@/components/powertrain/CurrentChart.vue'

// Everything needed while the car is moving, the headline numbers big enough
// to read from arm's length. Speed is a display format of motor RPM, so its
// severity stays keyed off the real rev limit rather than a figure derived
// through the wheel and gearing.
const { frame } = useTelemetry()

const formatMph = (rpm: number) => `${Math.round(motorRpmToMph(rpm))} mph`
const formatRpm = (rpm: number) => `${Math.round(rpm)} rpm`
const formatMotorAmps = (amps: number) => formatAmps(amps, 0)
// Rounded before formatting so a small negative reads as 0.0 rather than
// -0.0, same as formatAmps.
const formatKilowatts = (kilowatts: number) =>
    `${(Number(kilowatts.toFixed(1)) || 0).toFixed(1)} kW`
const formatCellVoltage = (volts: number) => formatVolts(volts, 3)
const formatLvVolts = (volts: number) => formatVolts(volts, 2)

// Battery power from the inverter's DC input voltage and the BMS's pack
// current. The BMS low-pass filters its current reading, so short peaks read
// a little low.
function batteryKilowatts(
    f: Pick<TelemetryFrame, 'inverter_input_voltage' | 'positive_current'>,
): number | undefined {
    if (f.inverter_input_voltage === undefined || f.positive_current === undefined) return
    return (f.inverter_input_voltage * f.positive_current) / 1000
}

const selectRpm = (f: TelemetryFrame) => f.motor_rpm
const selectDcVoltage = (f: TelemetryFrame) => f.inverter_input_voltage
const selectMotorTemperature = (f: TelemetryFrame) => f.motor_temperature
const selectInverterTemperature = (f: TelemetryFrame) => f.inverter_temperature
const selectPackCurrent = (f: TelemetryFrame) => f.positive_current
const selectMinCellVoltage = (f: TelemetryFrame) => f.min_cell_voltage
const selectMaxCellTemperature = (f: TelemetryFrame) => f.max_cell_temperature
const selectLvMin = (f: TelemetryFrame) => f.lvs_min_voltage
const selectLvMax = (f: TelemetryFrame) => f.lvs_max_voltage

const kilowatts = computed(() => batteryKilowatts(frame))

// Running totals, both estimates. Distance is what the wheels turned, so it
// counts wheelspin and running on stands too, and energy is net of regen.
const distance = useRunningTotal((f) =>
    f.motor_rpm === undefined ? undefined : (Math.abs(f.motor_rpm) / 60) * METRES_PER_MOTOR_REV,
)
const energy = useRunningTotal(batteryKilowatts)
const distanceLabel = computed(() => {
    const metres = distance.total.value
    return metres < 1000 ? `${Math.round(metres)} m` : `${(metres / 1000).toFixed(2)} km`
})
// Totalled in kilowatt-seconds.
const energyLabel = computed(() => {
    const wattHours = energy.total.value / 3.6
    return Math.abs(wattHours) < 1000
        ? `${Math.round(wattHours)} Wh`
        : `${(wattHours / 1000).toFixed(2)} kWh`
})

// From the lowest cell, since that's the one that ends the run, and only
// while the pack is near rest: under load the cells sag and it would read low.
const stateOfCharge = ref<number>()
watchEffect(() => {
    const { min_cell_voltage: volts, positive_current: amps } = frame
    if (volts === undefined || amps === undefined) return
    if (Math.abs(amps) <= STATE_OF_CHARGE_MAX_CURRENT) {
        stateOfCharge.value = estimateStateOfCharge(volts)
    }
})
const stateOfChargeLabel = computed(() =>
    stateOfCharge.value === undefined ? '—' : `${Math.round(stateOfCharge.value)}%`,
)
const stateOfChargeSeverityValue = computed(() =>
    stateOfCharge.value === undefined ? undefined : stateOfChargeSeverity(stateOfCharge.value),
)

// Same chain as the powertrain: the inverter, and the pedal via front
// distribution. `undefined` only once neither has reported.
const online = computed<boolean | undefined>(() => {
    const inverter = isFlagOnline(frame.online_flags, 'INVERTER_ONLINE')
    const front = isFlagOnline(frame.online_flags, 'FRONT_ONLINE')
    if (inverter === false || front === false) return false
    if (inverter === undefined && front === undefined) return undefined
    return true
})
</script>

<template>
    <Section :online="online" title="Drive">
        <template #header>
            <PumpIndicator />
        </template>
        <AutoGrid min="14rem">
            <MetricTile
                name="Speed"
                large
                :value="frame.motor_rpm"
                :format="formatMph"
                :select="selectRpm"
                :severity-of="rpmSeverity"
            />
            <BarTile
                name="Motor current"
                large
                :value="frame.motor_current"
                :max="MOTOR_CURRENT_MAX"
                :format="formatMotorAmps"
            />
            <MetricTile
                name="Battery power"
                large
                :value="kilowatts"
                :format="formatKilowatts"
                :select="batteryKilowatts"
            />
            <Tile
                title="Battery level"
                large
                :value="stateOfChargeLabel"
                :severity="stateOfChargeSeverityValue"
            >
                <template #sub>
                    <ExplanationSub
                        :text="`Estimate from the lowest cell, held above ${STATE_OF_CHARGE_MAX_CURRENT} A`"
                    />
                </template>
            </Tile>
        </AutoGrid>

        <AutoGrid min="12rem">
            <MetricTile
                name="Motor RPM"
                :value="frame.motor_rpm"
                :format="formatRpm"
                :select="selectRpm"
                :severity-of="rpmSeverity"
            />
            <MetricTile
                name="TS voltage"
                :value="frame.inverter_input_voltage"
                :format="formatVolts"
                :select="selectDcVoltage"
                :severity-of="inverterInputVoltageSeverity"
            />
            <MetricTile
                name="Motor temperature"
                :value="frame.motor_temperature"
                :format="formatCelsius"
                :select="selectMotorTemperature"
                :severity-of="motorTemperatureSeverity"
            />
            <MetricTile
                name="Inverter temperature"
                :value="frame.inverter_temperature"
                :format="formatCelsius"
                :select="selectInverterTemperature"
                :severity-of="inverterTemperatureSeverity"
            />
        </AutoGrid>

        <AutoGrid min="12rem">
            <MetricTile
                name="Battery current"
                :value="frame.positive_current"
                :format="formatAmps"
                :select="selectPackCurrent"
                :severity-of="packCurrentSeverity"
            />
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
            <MetricTile
                name="LVS voltage"
                :value="frame.lvs_min_voltage"
                :format="formatLvVolts"
                :select="selectLvMin"
                :select-max="selectLvMax"
                :severity-of="lvVoltageSeverity"
            />
        </AutoGrid>

        <AutoGrid min="14rem">
            <BarTile
                name="Pedal travel"
                :value="frame.pedal_travel"
                :max="100"
                :format="formatPercent"
            />
            <InverterFaultTile :fault="frame.inverter_fault" />
            <TotalTile
                title="Distance (estimate)"
                :value="distanceLabel"
                :since="distance.since.value"
                @reset="distance.reset"
            />
            <TotalTile
                title="Energy used"
                :value="energyLabel"
                :since="energy.since.value"
                @reset="energy.reset"
            />
        </AutoGrid>

        <div class="charts">
            <Tile title="RPM history" chart>
                <RpmChart />
            </Tile>
            <Tile title="Current history" chart>
                <CurrentChart />
            </Tile>
        </div>
    </Section>
</template>

<style scoped>
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
