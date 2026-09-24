<script setup lang="ts">
import { computed } from 'vue'
import type { BmsMasterErrorFlag } from '@/telemetry'
import { BMS_ERROR_META, BMS_ERROR_ORDER, bmsErrorSeverity } from '@/domain/bms'
import Tile from '@/components/Tile.vue'
import FlagsList from '@/components/FlagsList.vue'
import type { FlagRow } from '@/components/FlagsList.vue'

const props = defineProps<{
    flags?: readonly BmsMasterErrorFlag[]
}>()

// Every master error is a fault, so none of FlagsList's tone/tag/dimming
// machinery applies here — that's all precharge's, whose flags include
// informational and latched ones.
const rows = computed<FlagRow[]>(() =>
    BMS_ERROR_ORDER.filter((flag) => props.flags?.includes(flag)).map((flag) => ({
        key: flag,
        label: BMS_ERROR_META[flag].label,
        description: BMS_ERROR_META[flag].description,
        tone: 'fault',
    })),
)

const severity = computed(() => bmsErrorSeverity(props.flags))
</script>

<template>
    <Tile title="Master errors" accent :severity="severity">
        <FlagsList :rows="rows" :empty-text="flags === undefined ? 'No signal' : 'No errors'" />
    </Tile>
</template>
