<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import { isFlagOnline } from '@/telemetry'
import StaleSection from '@/components/StaleSection.vue'
import SectionPanel from '@/components/SectionPanel.vue'
import PrechargeStateStepper from '@/components/precharge/PrechargeStateStepper.vue'
import PrechargeFlagsList from '@/components/precharge/PrechargeFlagsList.vue'
import PrechargeRelayStates from '@/components/precharge/PrechargeRelayStates.vue'
import PrechargeVoltages from '@/components/precharge/PrechargeVoltages.vue'
import PrechargeVoltageChart from '@/components/precharge/PrechargeVoltageChart.vue'

const { frame } = useTelemetry()
</script>

<template>
    <StaleSection :online="isFlagOnline(frame.online_flags, 'PRECHARGE_ONLINE')" title="Precharge">
        <PrechargeStateStepper :state="frame.precharge_state" />

        <div class="body">
            <div class="status-col">
                <SectionPanel title="Voltages">
                    <PrechargeVoltages
                        :prchg-voltage="frame.precharge_prchg_voltage"
                        :ts-voltage="frame.precharge_ts_voltage"
                    />
                </SectionPanel>
                <SectionPanel title="Relay States">
                    <PrechargeRelayStates :relays="frame.precharge_relay_states" />
                </SectionPanel>
                <SectionPanel title="Flags">
                    <PrechargeFlagsList
                        :flags="frame.precharge_error_flags"
                        :state="frame.precharge_state"
                    />
                </SectionPanel>
            </div>

            <SectionPanel title="Voltage history" chart>
                <PrechargeVoltageChart />
            </SectionPanel>
        </div>
    </StaleSection>
</template>

<style scoped>
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

@media (max-width: 47.5em) {
    .body {
        grid-template-columns: 1fr;
    }
}
</style>
