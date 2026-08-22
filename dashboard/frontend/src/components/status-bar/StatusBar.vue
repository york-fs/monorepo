<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import ConnectionStatusTile from '@/components/status-bar/ConnectionStatusTile.vue'
import { isFlagOnline } from '@/telemetry'

const { frame } = useTelemetry()
</script>

<template>
    <div class="status-bar">
        <ConnectionStatusTile name="Rear distribution" />
        <ConnectionStatusTile
            name="Front distribution"
            has-own-signal
            :online="isFlagOnline(frame.online_flags, 'FRONT_ONLINE')"
        />
        <ConnectionStatusTile
            name="BMS"
            has-own-signal
            :online="isFlagOnline(frame.online_flags, 'BMS_ONLINE')"
        />
        <ConnectionStatusTile
            name="Precharge"
            has-own-signal
            :online="isFlagOnline(frame.online_flags, 'PRECHARGE_ONLINE')"
        />
        <ConnectionStatusTile
            name="Inverter"
            has-own-signal
            :online="isFlagOnline(frame.online_flags, 'INVERTER_ONLINE')"
        />
    </div>
</template>

<style scoped>
.status-bar {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(10.5rem, 1fr));
    gap: 1rem;
}
</style>
