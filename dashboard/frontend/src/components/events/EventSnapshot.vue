<script setup lang="ts">
import { computed } from 'vue'
import {
    EVENT_CATEGORY_LABELS,
    codeLabel,
    snapshotProblems,
    snapshotReadings,
    type EventSnapshot,
} from '@/domain/events'

// What the car looked like when something deactivated: everything that was
// set at that moment, and the problems raised in the couple of seconds
// before. The lead-up matters because the flag that actually caused a TS
// drop has often already cleared by the time the precharge reacts — e.g. a
// brief BMS fault that made the front distribution withdraw its TS request.
const props = defineProps<{
    snapshot: EventSnapshot
    /** Hide the readings, for the summary tiles. */
    compact?: boolean
}>()

const problems = computed(() => snapshotProblems(props.snapshot.frame))
const readings = computed(() => snapshotReadings(props.snapshot.frame))

// The backend sends newest first; oldest first reads as a timeline.
const leadUp = computed(() => [...props.snapshot.lead_up].reverse())

function offset(beforeMs: number): string {
    return beforeMs === 0 ? 'same frame' : `−${beforeMs} ms`
}
</script>

<template>
    <div class="snapshot">
        <div v-if="leadUp.length > 0" class="block">
            <p class="label">Raised in the 2 s before</p>
            <ul class="lead-up">
                <li
                    v-for="(entry, index) in leadUp"
                    :key="index"
                    :class="{ routine: entry.severity === 'info' }"
                >
                    <span class="offset">{{ offset(entry.before_ms) }}</span>
                    <span>
                        <span class="category">{{ EVENT_CATEGORY_LABELS[entry.category] }}</span>
                        {{ codeLabel(entry.category, entry.code) }}
                    </span>
                </li>
            </ul>
        </div>
        <div class="block">
            <p class="label">Set at the time</p>
            <p v-if="problems.length === 0" class="none">Nothing</p>
            <dl v-else class="pairs">
                <template v-for="group in problems" :key="group.label">
                    <dt>{{ group.label }}</dt>
                    <dd>{{ group.items.join(', ') }}</dd>
                </template>
            </dl>
        </div>
        <div v-if="!compact && readings.length > 0" class="block">
            <p class="label">Readings</p>
            <dl class="pairs">
                <template v-for="item in readings" :key="item.label">
                    <dt>{{ item.label }}</dt>
                    <dd class="number">{{ item.value }}</dd>
                </template>
            </dl>
        </div>
    </div>
</template>

<style scoped>
.snapshot {
    display: grid;
    gap: 0.75rem;
    font-size: 0.8125rem;
}

.block {
    display: grid;
    gap: var(--gap-tight);
}

.label {
    margin: 0;
    font-size: 0.7rem;
    font-weight: 600;
    text-transform: uppercase;
    color: var(--ink-muted);
}

.none {
    margin: 0;
    color: var(--ink-muted);
}

.lead-up {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    gap: 0.125rem;
}

.lead-up li {
    display: grid;
    grid-template-columns: 5rem minmax(0, 1fr);
    column-gap: var(--gap-items);
    color: var(--status-critical-text);
}

.lead-up li.routine {
    color: var(--ink-muted);
}

.offset {
    font-variant-numeric: tabular-nums;
    color: var(--ink-muted);
}

.category {
    font-weight: 600;
}

.pairs {
    margin: 0;
    display: grid;
    /* Capped so a long label wraps instead of squeezing the values into a
       sliver in the narrow summary tiles. */
    grid-template-columns: fit-content(45%) minmax(0, 1fr);
    column-gap: var(--gap-inline);
    row-gap: 0.125rem;
}

dt {
    color: var(--ink-muted);
}

dd {
    margin: 0;
    color: var(--ink-primary);
}

.number {
    font-variant-numeric: tabular-nums;
}
</style>
