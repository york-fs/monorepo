<script setup lang="ts">
import Badge from '@/components/Badge.vue'
import StatusBar from '@/components/status-bar/StatusBar.vue'
import DistributionSection from '@/components/distribution/DistributionSection.vue'
import PowertrainSection from '@/components/powertrain/PowertrainSection.vue'
import BmsSection from '@/components/bms/BmsSection.vue'
import PrechargeSection from '@/components/precharge/PrechargeSection.vue'
import { isDemoMode } from '@/demo'

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
        <BmsSection />
        <PrechargeSection />
    </main>
</template>

<style scoped>
.home {
    display: grid;
    gap: var(--gap-sections);
    max-width: 1200px;
    margin: 0 auto;
    padding: 1.5rem 1.25rem;
}

.title-row {
    display: flex;
    align-items: center;
    gap: var(--gap-inline);
}
</style>
