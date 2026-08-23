<script setup lang="ts">
import StatusBar from '@/components/status-bar/StatusBar.vue'
import DistributionSection from '@/components/distribution/DistributionSection.vue'
import PrechargeSection from '@/components/precharge/PrechargeSection.vue'
import { isDemoMode } from '@/demo'
</script>

<template>
    <div class="home">
        <div class="title-row">
            <h1>YFS-03 Telemetry</h1>
            <span v-if="isDemoMode()" class="demo-badge">Demo mode — synthetic data</span>
        </div>
        <StatusBar />
        <DistributionSection />
        <PrechargeSection />
    </div>
</template>

<style scoped>
.home {
    max-width: 68.75rem;
    margin: 0 auto;
    display: grid;
    gap: 1.75rem;
    padding: 1.5rem 1.25rem;
}

.title-row {
    display: flex;
    align-items: center;
    gap: 0.75rem;
}

h1 {
    /* The UA default (~2em, bold, with its own top/bottom margin) is both
       oversized next to the rest of the type scale and adds extra space on
       top of .home's grid gap, since grid items don't collapse margins. */
    font-size: 1.375rem;
    font-weight: 650;
    letter-spacing: -0.01em;
    margin: 0;
    color: var(--ink-primary);
}

.demo-badge {
    font-size: 0.75rem;
    font-weight: 600;
    padding: 0.15rem 0.5rem;
    border-radius: 0.25rem;
    background: var(--status-warning);
    color: var(--status-warning-text);
}
</style>
