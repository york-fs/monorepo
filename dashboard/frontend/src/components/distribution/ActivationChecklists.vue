<script setup lang="ts">
import { computed } from 'vue'
import { isRtdPreventionFlagSet, isTsPreventionFlagSet } from '@/telemetry'
import type { RtdPreventionFlag, TsPreventionFlag } from '@/telemetry'
import {
    RTD_PREVENTION_FLAGS,
    RTD_PREVENTION_LABELS,
    TS_PREVENTION_FLAGS,
    TS_PREVENTION_LABELS,
} from '@/domain/prevention'
import PreventionChecklistPanel from '@/components/distribution/PreventionChecklistPanel.vue'

const props = defineProps<{
    tsPreventionFlags?: readonly TsPreventionFlag[]
    rtdPreventionFlags?: readonly RtdPreventionFlag[]
}>()

// Checklist "ok" is the inverse of "flag set" — these are prevention flags,
// so a set flag means the condition is failing, not satisfied.
function negate(set: boolean | undefined): boolean | undefined {
    return set === undefined ? undefined : !set
}

const tsRows = computed(() =>
    TS_PREVENTION_FLAGS.map((flag) => ({
        key: flag,
        label: TS_PREVENTION_LABELS[flag],
        ok: negate(isTsPreventionFlagSet(props.tsPreventionFlags, flag)),
    })),
)

const rtdRows = computed(() =>
    RTD_PREVENTION_FLAGS.map((flag) => ({
        key: flag,
        label: RTD_PREVENTION_LABELS[flag],
        ok: negate(isRtdPreventionFlagSet(props.rtdPreventionFlags, flag)),
    })),
)

// Suppress the RTD panel while TS isn't active, rather than showing two
// overlapping problems (RTD blocked partly *because* TS isn't active) as if
// they were independent. Only suppress once we actually know TS is blocked —
// no signal yet shouldn't read as "suppressed".
const tsBlocked = computed(() => (props.tsPreventionFlags?.length ?? 0) > 0)

// Accent colour for each panel: green once nothing is blocking, amber if the
// only thing left is that activation hasn't been requested yet, red for any
// actual fault/offline/state condition still blocking. `undefined` (no
// signal yet) is left uncoloured, same convention as the rest of the app.
function activationSeverity(
    flags: readonly string[] | undefined,
): 'good' | 'warning' | 'critical' | undefined {
    if (flags === undefined) return undefined
    if (flags.length === 0) return 'good'
    if (flags.length === 1 && flags[0] === 'NOT_REQUESTED') return 'warning'
    return 'critical'
}

const tsSeverity = computed(() => activationSeverity(props.tsPreventionFlags))
const rtdSeverity = computed(() => activationSeverity(props.rtdPreventionFlags))
</script>

<template>
    <div class="checklists">
        <PreventionChecklistPanel title="TS activation" :rows="tsRows" :severity="tsSeverity" />
        <PreventionChecklistPanel
            title="RTD activation"
            :rows="rtdRows"
            :severity="rtdSeverity"
            :suppressed="tsBlocked"
            suppressed-note="TS not active"
        />
    </div>
</template>

<style scoped>
.checklists {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(14rem, 1fr));
    gap: 1rem;
}
</style>
