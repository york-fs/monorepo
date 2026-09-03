<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import {
    inverterFaultExplanation,
    inverterFaultLabel,
    inverterFaultSeverity,
} from '@/domain/powertrain'
import type { InverterFaultCode } from '@/telemetry'

const props = defineProps<{
    fault?: InverterFaultCode
}>()

const severity = computed(() =>
    props.fault === undefined ? undefined : inverterFaultSeverity(props.fault),
)
const label = computed(() => (props.fault === undefined ? '—' : inverterFaultLabel(props.fault)))
const explanation = computed(() =>
    props.fault === undefined ? undefined : inverterFaultExplanation(props.fault),
)
</script>

<template>
    <AccentTile name="Inverter fault" :severity="severity">
        {{ label }}
        <template #sub>
            <span v-if="explanation" class="explanation">{{ explanation }}</span>
        </template>
    </AccentTile>
</template>

<style scoped>
.explanation {
    font-size: 0.8125rem;
    color: var(--ink-muted);
}
</style>
