<script setup lang="ts">
import { computed } from 'vue'
import { useTelemetry } from '@/composables/useTelemetry'
import { useMinMax } from '@/composables/useMinMax'
import { isFlagOnline } from '@/telemetry'
import { inverterTemperatureSeverity, motorTemperatureSeverity } from '@/domain/powertrain'
import StaleSection from '@/components/StaleSection.vue'
import SpeedTile from '@/components/powertrain/SpeedTile.vue'
import DcVoltageTile from '@/components/powertrain/DcVoltageTile.vue'
import TemperatureTile from '@/components/powertrain/TemperatureTile.vue'
import InverterFaultTile from '@/components/powertrain/InverterFaultTile.vue'
import RpmChart from '@/components/powertrain/RpmChart.vue'
import CurrentChart from '@/components/powertrain/CurrentChart.vue'
import PedalTravelTile from '@/components/powertrain/PedalTravelTile.vue'

const { frame } = useTelemetry()

const inverterTempSeverity = computed(() =>
    frame.inverter_temperature === undefined
        ? undefined
        : inverterTemperatureSeverity(frame.inverter_temperature),
)
const motorTempSeverity = computed(() =>
    frame.motor_temperature === undefined
        ? undefined
        : motorTemperatureSeverity(frame.motor_temperature),
)

const inverterTempRange = useMinMax((f) => f.inverter_temperature)
const inverterTempEverMinSeverity = computed(() =>
    inverterTempRange.everMin.value === undefined
        ? undefined
        : inverterTemperatureSeverity(inverterTempRange.everMin.value),
)
const inverterTempEverMaxSeverity = computed(() =>
    inverterTempRange.everMax.value === undefined
        ? undefined
        : inverterTemperatureSeverity(inverterTempRange.everMax.value),
)

const motorTempRange = useMinMax((f) => f.motor_temperature)
const motorTempEverMinSeverity = computed(() =>
    motorTempRange.everMin.value === undefined
        ? undefined
        : motorTemperatureSeverity(motorTempRange.everMin.value),
)
const motorTempEverMaxSeverity = computed(() =>
    motorTempRange.everMax.value === undefined
        ? undefined
        : motorTemperatureSeverity(motorTempRange.everMax.value),
)

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
    <StaleSection :online="online">
        <template #header>
            <h2>Powertrain</h2>
        </template>

        <section class="powertrain">
            <div class="summary">
                <div class="summary-row">
                    <SpeedTile :rpm="frame.motor_rpm" />
                    <InverterFaultTile :fault="frame.inverter_fault" />
                </div>
                <div class="summary-row">
                    <DcVoltageTile :volts="frame.inverter_input_voltage" />
                    <TemperatureTile
                        name="Inverter temperature"
                        :celsius="frame.inverter_temperature"
                        :severity="inverterTempSeverity"
                        :ever-min="inverterTempRange.everMin.value"
                        :ever-min-severity="inverterTempEverMinSeverity"
                        :ever-max="inverterTempRange.everMax.value"
                        :ever-max-severity="inverterTempEverMaxSeverity"
                    />
                    <TemperatureTile
                        name="Motor temperature"
                        :celsius="frame.motor_temperature"
                        :severity="motorTempSeverity"
                        :ever-min="motorTempRange.everMin.value"
                        :ever-min-severity="motorTempEverMinSeverity"
                        :ever-max="motorTempRange.everMax.value"
                        :ever-max-severity="motorTempEverMaxSeverity"
                    />
                </div>
            </div>

            <div class="charts">
                <div class="panel chart-panel">
                    <h3>RPM history</h3>
                    <RpmChart />
                </div>
                <div class="panel chart-panel">
                    <h3>Current history</h3>
                    <CurrentChart />
                </div>
            </div>

            <div class="apps">
                <h3>APPS</h3>
                <div class="summary-row">
                    <PedalTravelTile :travel="frame.pedal_travel" />
                </div>
            </div>
        </section>
    </StaleSection>
</template>

<style scoped>
.powertrain {
    display: grid;
    gap: 1.25rem;
}

.summary {
    display: grid;
    gap: 1rem;
}

.summary-row {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(10.5rem, 1fr));
    gap: 1rem;
}

.charts {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1rem;
}

.panel {
    background: var(--surface-card);
    border: 1px solid var(--border);
    border-radius: 0.375rem;
    padding: 0.875rem 1rem;
}

.chart-panel {
    display: grid;
    grid-template-rows: auto 1fr;
}

@media (max-width: 47.5em) {
    .charts {
        grid-template-columns: 1fr;
    }
}
</style>
