<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import { isFlagOnline } from '@/telemetry'
import type { TelemetryFrame } from '@/telemetry'
import { lvVoltageSeverity } from '@/domain/lvVoltage'
import { formatVolts } from '@/utils/formatVolts'
import StaleSection from '@/components/StaleSection.vue'
import MetricTile from '@/components/MetricTile.vue'
import FuseGrid from '@/components/distribution/FuseGrid.vue'
import ShutdownCauseTile from '@/components/distribution/ShutdownCauseTile.vue'
import ActivationChecklists from '@/components/distribution/ActivationChecklists.vue'

const { frame } = useTelemetry()

// The live reading is the *minimum* across all fuses — that's what signals
// brownout risk. The maximum only ever appears as the top of the ever-range,
// and comes from its own wire field, hence the two selectors. See
// plan/DISTRIBUTION.md.
const formatLvVolts = (volts: number) => formatVolts(volts, 2)
const selectLvMin = (f: TelemetryFrame) => f.lvs_min_voltage
const selectLvMax = (f: TelemetryFrame) => f.lvs_max_voltage
</script>

<template>
    <!-- Rear distribution has no `online_flags` entry of its own — it's
         what carries the telemetry link, so the overall link staleness
         `StaleSection` already checks covers it. Passing front's flag here
         means this section goes stale if either board does. -->
    <StaleSection :online="isFlagOnline(frame.online_flags, 'FRONT_ONLINE')" title="Distribution">
        <div class="summary">
            <MetricTile
                name="LV system"
                :value="frame.lvs_min_voltage"
                :format="formatLvVolts"
                :select="selectLvMin"
                :select-max="selectLvMax"
                :severity-of="lvVoltageSeverity"
            />
            <ShutdownCauseTile :cause="frame.shutdown_open_cause" />
        </div>
        <FuseGrid :fuses="frame.fuses" />
        <ActivationChecklists
            :ts-prevention-flags="frame.ts_prevention_flags"
            :rtd-prevention-flags="frame.rtd_prevention_flags"
        />
    </StaleSection>
</template>

<style scoped>
.summary {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(12rem, 1fr));
    gap: 1rem;
}
</style>
