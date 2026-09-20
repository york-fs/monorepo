<script setup lang="ts">
import type { Severity } from '@/domain/severity'
import { computed } from 'vue'
import { isRtdPreventionFlagSet, isTsPreventionFlagSet } from '@/telemetry'
import type { RtdPreventionFlag, TsPreventionFlag } from '@/telemetry'
import { RTD_PREVENTION_CONDITIONS, TS_PREVENTION_CONDITIONS } from '@/domain/prevention'
import type { PreventionCondition } from '@/domain/prevention'
import AutoGrid from '@/components/AutoGrid.vue'
import ChecklistTile from '@/components/distribution/ChecklistTile.vue'

const props = defineProps<{
    tsPreventionFlags?: readonly TsPreventionFlag[]
    rtdPreventionFlags?: readonly RtdPreventionFlag[]
}>()

// Checklist "ok" is the inverse of "flag set" — these are prevention flags,
// so a set flag means the condition is failing, not satisfied.
function negate(set: boolean | undefined): boolean | undefined {
    return set === undefined ? undefined : !set
}

function rows<T extends string>(
    conditions: PreventionCondition<T>[],
    isSet: (flags: readonly T[] | undefined, flag: T) => boolean | undefined,
    flags: readonly T[] | undefined,
) {
    return conditions.map(({ flag, label }) => ({ flag, label, ok: negate(isSet(flags, flag)) }))
}

const tsRows = computed(() =>
    rows(TS_PREVENTION_CONDITIONS, isTsPreventionFlagSet, props.tsPreventionFlags),
)

const rtdRows = computed(() =>
    rows(RTD_PREVENTION_CONDITIONS, isRtdPreventionFlagSet, props.rtdPreventionFlags),
)

// Accent colour for each tile: green once nothing is blocking, amber if the
// only thing left is that activation hasn't been requested yet, red for any
// actual fault/offline/state condition still blocking. `undefined` (no
// signal yet) is left uncoloured, same convention as the rest of the app.
function activationSeverity(flags: readonly string[] | undefined): Severity | undefined {
    if (flags === undefined) return undefined
    if (flags.length === 0) return 'good'
    if (flags.length === 1 && flags[0] === 'NOT_REQUESTED') return 'warning'
    return 'critical'
}

const tsSeverity = computed(() => activationSeverity(props.tsPreventionFlags))
const rtdSeverity = computed(() => activationSeverity(props.rtdPreventionFlags))
</script>

<template>
    <div>
        <h3>Activation</h3>
        <AutoGrid min="14rem">
            <ChecklistTile title="TS activation" :rows="tsRows" :severity="tsSeverity" />
            <ChecklistTile title="RTD activation" :rows="rtdRows" :severity="rtdSeverity" />
        </AutoGrid>
    </div>
</template>
