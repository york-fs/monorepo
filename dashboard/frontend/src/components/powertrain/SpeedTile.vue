<script setup lang="ts">
import { computed } from 'vue'
import AccentTile from '@/components/AccentTile.vue'
import MinMaxSub from '@/components/MinMaxSub.vue'
import { useMinMax } from '@/composables/useMinMax'
import { motorRpmToMph } from '@/domain/powertrain'

const props = defineProps<{
    rpm?: number
}>()

const mph = computed(() => (props.rpm === undefined ? undefined : motorRpmToMph(props.rpm)))

const { everMin, everMax } = useMinMax((frame) =>
    frame.motor_rpm === undefined ? undefined : motorRpmToMph(frame.motor_rpm),
)

function fmt(value: number | undefined) {
    return value === undefined ? '—' : `${Math.round(value)} mph`
}
</script>

<template>
    <!-- No severity yet — see plan/PLAN.md, pending an RPM redline to key it off. -->
    <AccentTile name="Speed">
        {{ fmt(mph) }}
        <template #sub>
            <MinMaxSub :min-label="fmt(everMin)" :max-label="fmt(everMax)" />
        </template>
    </AccentTile>
</template>
