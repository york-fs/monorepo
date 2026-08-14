<script setup lang="ts">
import StatusBar from '@/components/status-bar/StatusBar.vue'
import DistributionSection from '@/components/distribution/DistributionSection.vue'
import PrechargeSection from '@/components/precharge/PrechargeSection.vue'
</script>

<template>
    <div class="home">
        <h1>YFS-03 Telemetry</h1>
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
</style>
