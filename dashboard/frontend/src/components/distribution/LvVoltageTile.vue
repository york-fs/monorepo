<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import MinMaxSub from '@/components/MinMaxSub.vue'
import { useMinMax } from '@/composables/useMinMax'
import { lvVoltageSeverity } from '@/domain/lvVoltage'

const props = defineProps<{
    minVoltage?: number
}>()

// Two independent trackers, each contributing only the extremum it actually
// wants — that's the value that matters for each field (see PLAN.md): the
// lowest `lvs_min_voltage` and the highest `lvs_max_voltage` ever seen.
const { everMin } = useMinMax((frame) => frame.lvs_min_voltage)
const { everMax } = useMinMax((frame) => frame.lvs_max_voltage)

const severity = computed(() =>
    props.minVoltage === undefined ? undefined : lvVoltageSeverity(props.minVoltage),
)
const everMinSeverity = computed(() =>
    everMin.value === undefined ? undefined : lvVoltageSeverity(everMin.value),
)
const everMaxSeverity = computed(() =>
    everMax.value === undefined ? undefined : lvVoltageSeverity(everMax.value),
)

function fmt(volts: number | undefined) {
    return volts === undefined ? '—' : `${volts.toFixed(2)} V`
}
</script>

<template>
    <AccentTile name="LV system" :severity="severity">
        {{ fmt(minVoltage) }}
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

<style scoped>
:deep(.tile) {
    min-width: 12rem;
}
</style>
