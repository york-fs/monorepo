<script setup lang="ts">
import AccentTile from '@/components/AccentTile.vue'
import MinMaxSub from '@/components/MinMaxSub.vue'

defineProps<{
    name: string
    celsius?: number
    severity?: 'good' | 'warning' | 'critical'
    everMin?: number
    everMinSeverity?: 'good' | 'warning' | 'critical'
    everMax?: number
    everMaxSeverity?: 'good' | 'warning' | 'critical'
}>()

function fmt(value: number | undefined) {
    return value === undefined ? '—' : `${Math.round(value)}°C`
}
</script>

<template>
    <AccentTile :name="name" :severity="severity">
        {{ fmt(celsius) }}
        <template #sub>
            <MinMaxSub
                :min-label="fmt(everMin)"
                :min-severity="everMinSeverity"
                :max-label="fmt(everMax)"
                :max-severity="everMaxSeverity"
            />
        </template>
    </AccentTile>
</template>
