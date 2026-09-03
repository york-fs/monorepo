<script setup lang="ts">
import AccentTile from '@/components/AccentTile.vue'
import { formatPercent } from '@/utils/formatPercent'

defineProps<{
    travel?: number
}>()
</script>

<template>
    <AccentTile name="Pedal travel">
        {{ formatPercent(travel) }}
    </AccentTile>
</template>
