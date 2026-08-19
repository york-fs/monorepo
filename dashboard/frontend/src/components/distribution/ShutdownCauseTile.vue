<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import { shutdownOpenCauseLabel, shutdownOpenCauseSeverity } from '@/domain/shutdown'
import type { ShutdownOpenCause } from '@/types/telemetry'

const props = defineProps<{
    cause?: ShutdownOpenCause
}>()

const severity = computed(() =>
    props.cause === undefined ? undefined : shutdownOpenCauseSeverity(props.cause),
)
const label = computed(() =>
    props.cause === undefined ? '—' : shutdownOpenCauseLabel(props.cause),
)
</script>

<template>
    <AccentTile name="Shutdown open cause" :severity="severity">
        {{ label }}
    </AccentTile>
</template>
