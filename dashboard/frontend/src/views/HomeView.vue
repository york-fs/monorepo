<script setup lang="ts">
import Badge from '@/components/Badge.vue'
import StatusBar from '@/components/status-bar/StatusBar.vue'
import DistributionSection from '@/components/distribution/DistributionSection.vue'
import PowertrainSection from '@/components/powertrain/PowertrainSection.vue'
import PrechargeSection from '@/components/precharge/PrechargeSection.vue'
import { isDemoMode } from '@/demo'

// Read once — it's a URL check, not reactive state, so calling it from the
// template would re-parse the query string on every render.
const demoMode = isDemoMode()
</script>

<template>
    <main class="home">
        <div class="title-row">
            <h1>YFS-03 Telemetry</h1>
            <Badge v-if="demoMode">Demo mode — synthetic data</Badge>
        </div>
        <StatusBar />
        <DistributionSection />
        <PowertrainSection />
        <PrechargeSection />
    </main>
</template>

<style scoped>
.home {
    max-width: 68.75rem;
    margin: 0 auto;
    display: grid;
    gap: var(--gap-sections);
    padding: 1.5rem 1.25rem;
}

.title-row {
    display: flex;
    align-items: center;
    gap: var(--gap-inline);
}

h1 {
    /* The UA default (~2em, bold, with its own top/bottom margin) is both
       oversized next to the rest of the type scale and adds extra space on
       top of .home's grid gap, since grid items don't collapse margins. */
    font-size: 1.5rem;
    font-weight: 650;
    letter-spacing: -0.01em;
    margin: 0;
    color: var(--ink-primary);
}
</style>
