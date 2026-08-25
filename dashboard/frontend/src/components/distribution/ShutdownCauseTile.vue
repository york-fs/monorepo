<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import {
    shutdownOpenCauseExplanation,
    shutdownOpenCauseLabel,
    shutdownOpenCauseSeverity,
} from '@/domain/shutdown'
import type { ShutdownOpenCause } from '@/telemetry'

const props = defineProps<{
    cause?: ShutdownOpenCause
}>()

const severity = computed(() =>
    props.cause === undefined ? undefined : shutdownOpenCauseSeverity(props.cause),
)
const label = computed(() =>
    props.cause === undefined ? '—' : shutdownOpenCauseLabel(props.cause),
)
const explanation = computed(() =>
    props.cause === undefined ? undefined : shutdownOpenCauseExplanation(props.cause),
)
</script>

<template>
    <AccentTile name="Shutdown open cause" :severity="severity">
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
