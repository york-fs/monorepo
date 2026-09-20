<script setup lang="ts">
import { useTelemetry } from '@/composables/useTelemetry'
import ConnectionStatusTile from '@/components/status-bar/ConnectionStatusTile.vue'
import AutoGrid from '@/components/AutoGrid.vue'
import { isFlagOnline } from '@/telemetry'

const { frame } = useTelemetry()
</script>

<template>
    <AutoGrid>
        <ConnectionStatusTile name="Rear distribution" />
        <ConnectionStatusTile
            name="Front distribution"
            has-own-signal
            :online="isFlagOnline(frame.online_flags, 'FRONT_ONLINE')"
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
        <ConnectionStatusTile
            name="BMS"
            has-own-signal
            :online="isFlagOnline(frame.online_flags, 'BMS_ONLINE')"
        />
    </AutoGrid>
</template>
