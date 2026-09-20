<script setup lang="ts">
import { computed } from 'vue'
import Tile from '@/components/Tile.vue'
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
    <Tile title="Shutdown open cause" :value="label" :severity="severity">
        <template #sub>
            <ExplanationSub v-if="explanation" :text="explanation" />
        </template>
    </Tile>
</template>
