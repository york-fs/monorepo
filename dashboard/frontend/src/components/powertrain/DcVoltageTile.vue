<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import MinMaxSub from '@/components/MinMaxSub.vue'
import { useMinMax } from '@/composables/useMinMax'
import { inverterInputVoltageSeverity } from '@/domain/powertrain'
import { formatVolts } from '@/utils/formatVolts'

const props = defineProps<{
    volts?: number
}>()

const severity = computed(() =>
    props.volts === undefined ? undefined : inverterInputVoltageSeverity(props.volts),
)

const { everMin, everMax } = useMinMax((frame) => frame.inverter_input_voltage)

const everMinSeverity = computed(() =>
    everMin.value === undefined ? undefined : inverterInputVoltageSeverity(everMin.value),
)
const everMaxSeverity = computed(() =>
    everMax.value === undefined ? undefined : inverterInputVoltageSeverity(everMax.value),
)
</script>

<template>
    <AccentTile name="DC input voltage" :severity="severity">
        {{ formatVolts(volts) }}
        <template #sub>
            <MinMaxSub
                :min-label="formatVolts(everMin)"
                :min-severity="everMinSeverity"
                :max-label="formatVolts(everMax)"
                :max-severity="everMaxSeverity"
            />
        </template>
    </AccentTile>
</template>
