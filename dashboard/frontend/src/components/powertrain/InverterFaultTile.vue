<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import ExplanationSub from '@/components/ExplanationSub.vue'
import {
    inverterFaultExplanation,
    inverterFaultLabel,
    inverterFaultSeverity,
} from '@/domain/powertrain'
import type { InverterFaultCode } from '@/telemetry'

const props = defineProps<{
    fault?: InverterFaultCode
}>()

// The domain helpers each answer for a missing reading themselves.
const severity = computed(() => inverterFaultSeverity(props.fault))
const label = computed(() => inverterFaultLabel(props.fault))
const explanation = computed(() => inverterFaultExplanation(props.fault))
</script>

<template>
    <AccentTile name="Inverter fault" :severity="severity">
        {{ label }}
        <template #sub>
            <ExplanationSub v-if="explanation" :text="explanation" />
        </template>
    </AccentTile>
</template>
