<script setup lang="ts">
import { computed } from 'vue'
import type { PrechargeErrorFlag, PrechargeState } from '@/telemetry'
import { PRECHARGE_FLAG_META, isFlagLive } from '@/domain/precharge'
import FlagsList from '@/components/FlagsList.vue'
import type { FlagRow } from '@/components/FlagsList.vue'

const props = defineProps<{
    flags?: readonly PrechargeErrorFlag[]
    state?: PrechargeState
}>()

const KIND_ORDER = { fault: 0, waiting: 1, deactivation: 2 }

// Only `fault` flags read as faults — "waiting" and "deactivation" are
// informational, so they stay quiet. A flag that isn't live in the current
// state is left over from the previous precharge attempt, not something
// happening now, hence the dimming and the tag.
const rows = computed<FlagRow[]>(() =>
    (props.flags ?? [])
        .map((flag) => ({ flag, meta: PRECHARGE_FLAG_META[flag] }))
        .sort((a, b) => KIND_ORDER[a.meta.kind] - KIND_ORDER[b.meta.kind])
        .map(({ flag, meta }) => {
            const live = isFlagLive(flag, props.state)
            return {
                key: flag,
                label: meta.label,
                description: meta.description,
                tone: meta.kind === 'fault' ? ('fault' as const) : ('neutral' as const),
                tag: live ? undefined : 'last attempt',
                dimmed: !live,
            }
        }),
)
</script>

<template>
    <FlagsList :rows="rows" empty-text="No flags set" />
</template>
