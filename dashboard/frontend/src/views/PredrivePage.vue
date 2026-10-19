<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import Section from '@/components/Section.vue'
import ActivationChecklists from '@/components/distribution/ActivationChecklists.vue'
import PrechargeSection from '@/components/precharge/PrechargeSection.vue'

const { frame } = useTelemetry()
</script>

<template>
    <Section title="Activation">
        <ActivationChecklists
            :ts-prevention-flags="frame.ts_prevention_flags"
            :rtd-prevention-flags="frame.rtd_prevention_flags"
        />
    </Section>
    <PrechargeSection />
</template>
