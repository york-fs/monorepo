<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import ConnectionStatusTile from '@/components/status-bar/ConnectionStatusTile.vue'

const { frame } = useTelemetry()
</script>

<template>
    <div class="status-bar">
        <ConnectionStatusTile name="Rear distribution" />
        <ConnectionStatusTile name="Precharge" :online="frame.precharge_online" />
    </div>
</template>

<style scoped>
.status-bar {
    display: flex;
    flex-wrap: wrap;
    gap: 1rem;
}
</style>
