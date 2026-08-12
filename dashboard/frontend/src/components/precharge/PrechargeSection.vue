<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import StaleSection from '@/components/StaleSection.vue'
import PrechargeStateStepper from '@/components/precharge/PrechargeStateStepper.vue'
import PrechargeFlagsList from '@/components/precharge/PrechargeFlagsList.vue'
import PrechargeRelayStates from '@/components/precharge/PrechargeRelayStates.vue'
import PrechargeVoltages from '@/components/precharge/PrechargeVoltages.vue'
import PrechargeVoltageChart from '@/components/precharge/PrechargeVoltageChart.vue'

const { frame } = useTelemetry()
</script>

<template>
    <StaleSection :online="frame.precharge_online">
        <template #header>
            <h2>Precharge</h2>
        </template>

        <section class="precharge">
            <PrechargeStateStepper :state="frame.precharge_state" />

            <div class="body">
                <div class="status-col">
                    <div class="panel">
                        <h3>Voltages</h3>
                        <PrechargeVoltages
                            :prchg-voltage="frame.precharge_prchg_voltage"
                            :ts-voltage="frame.precharge_ts_voltage"
                        />
                    </div>
                    <div class="panel">
                        <h3>Relays</h3>
                        <PrechargeRelayStates :relays="frame.precharge_relay_states" />
                    </div>
                    <div class="panel">
                        <h3>Flags</h3>
                        <PrechargeFlagsList
                            :flags="frame.precharge_error_flags"
                            :state="frame.precharge_state"
                        />
                    </div>
                </div>

                <div class="panel chart-panel">
                    <h3>Voltage history (up to 20 min)</h3>
                    <PrechargeVoltageChart />
                </div>
            </div>
        </section>
    </StaleSection>
</template>

<style scoped>
.precharge {
    display: grid;
    gap: 1.25rem;
}

h2 {
    font-size: 1rem;
    margin: 0;
    color: var(--ink-primary);
}

h3 {
    font-size: 0.75rem;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    color: var(--ink-muted);
    margin: 0 0 0.625rem;
}

.body {
    display: grid;
    grid-template-columns: minmax(15rem, 20rem) 1fr;
    gap: 1rem;
}

.status-col {
    display: grid;
    align-content: start;
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
    .body {
        grid-template-columns: 1fr;
    }
}
</style>
