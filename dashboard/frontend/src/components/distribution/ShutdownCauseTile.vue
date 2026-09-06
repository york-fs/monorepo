<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import ExplanationSub from '@/components/ExplanationSub.vue'
import {
    shutdownOpenCauseExplanation,
    shutdownOpenCauseLabel,
    shutdownOpenCauseSeverity,
} from '@/domain/shutdown'
import type { ShutdownOpenCause } from '@/telemetry'

const props = defineProps<{
    cause?: ShutdownOpenCause
}>()

// The domain helpers each answer for a missing reading themselves.
const severity = computed(() => shutdownOpenCauseSeverity(props.cause))
const label = computed(() => shutdownOpenCauseLabel(props.cause))
const explanation = computed(() => shutdownOpenCauseExplanation(props.cause))
</script>

<template>
    <AccentTile name="Shutdown open cause" :severity="severity">
        {{ label }}
        <template #sub>
            <ExplanationSub v-if="explanation" :text="explanation" />
        </template>
    </AccentTile>
</template>
