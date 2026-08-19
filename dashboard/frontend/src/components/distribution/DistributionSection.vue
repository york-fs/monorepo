<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import { isFlagOnline } from '@/types/telemetry'
import StaleSection from '@/components/StaleSection.vue'
import FuseGrid from '@/components/distribution/FuseGrid.vue'
import LvVoltageTile from '@/components/distribution/LvVoltageTile.vue'
import ShutdownCauseTile from '@/components/distribution/ShutdownCauseTile.vue'

const { frame } = useTelemetry()
</script>

<template>
    <!-- Rear distribution has no `online_flags` entry of its own — it's
         what carries the telemetry link, so the overall link staleness
         `StaleSection` already checks covers it. Passing front's flag here
         means this section goes stale if either board does. -->
    <StaleSection :online="isFlagOnline(frame.online_flags, 'FRONT_ONLINE')">
        <template #header>
            <h2>Distribution</h2>
        </template>

        <section class="distribution">
            <div class="summary">
                <LvVoltageTile :min-voltage="frame.lvs_min_voltage" />
                <ShutdownCauseTile :cause="frame.shutdown_open_cause" />
            </div>
            <FuseGrid :fuses="frame.fuses" />
        </section>
    </StaleSection>
</template>

<style scoped>
.distribution {
    display: grid;
    gap: 1.25rem;
}

.summary {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(12rem, 1fr));
    gap: 1rem;
}

h2 {
    font-size: 1rem;
    margin: 0;
    color: var(--ink-primary);
}
</style>
