<script setup lang="ts">
import { computed } from 'vue'
import Tile from '@/components/Tile.vue'
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
    <Tile title="Inverter fault" :value="label" :severity="severity">
        <template #sub>
            <ExplanationSub v-if="explanation" :text="explanation" />
        </template>
    </Tile>
</template>
