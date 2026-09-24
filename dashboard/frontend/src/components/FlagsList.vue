<script setup lang="ts">
// The "list of raised flags, each with an explanation" shape, shared by
// PrechargeFlagsList and BmsErrorList. Same split as Tile and MinMaxSub:
// callers own which flags are raised, what they're called, what order they
// come in and what a raised one means — this only knows how to draw a row.
//
// `tone`/`tag`/`dimmed` exist because precharge's flags aren't all faults:
// its "waiting" and "deactivation" kinds are informational, and a flag left
// over from the previous precharge attempt is shown greyed with a tag rather
// than as something happening now. The BMS's are all plain faults, and just
// leave those alone.
export interface FlagRow {
    /** Stable list key — the wire flag name, in practice. */
    key: string
    label: string
    description: string
    /** `fault` colours the dot and label red; `neutral` (default) is quiet. */
    tone?: 'fault' | 'neutral'
    /** Small uppercase note after the label, e.g. precharge's "last attempt". */
    tag?: string
    /** Dims the row — a flag that's shown but isn't live right now. */
    dimmed?: boolean
}

defineProps<{
    rows: readonly FlagRow[]
    /** Shown in place of the list when nothing is raised. */
    emptyText: string
}>()
</script>

<template>
    <div>
        <p v-if="rows.length === 0" class="empty">{{ emptyText }}</p>
        <ul v-else class="list">
            <li
                v-for="row in rows"
                :key="row.key"
                class="row"
                :class="[row.tone ?? 'neutral', { dimmed: row.dimmed }]"
            >
                <span class="dot" />
                <div class="text">
                    <span class="name">
                        {{ row.label }}
                        <span v-if="row.tag" class="tag">{{ row.tag }}</span>
                    </span>
                    <span class="description">{{ row.description }}</span>
                </div>
            </li>
        </ul>
    </div>
</template>

<style scoped>
.empty {
    margin: 0;
    font-size: 0.8125rem;
    color: var(--ink-muted);
}

.list {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    row-gap: var(--gap-rows);
}

.row {
    display: grid;
    grid-template-columns: auto 1fr;
    column-gap: var(--gap-rows);
}

.dot {
    width: 0.5rem;
    height: 0.5rem;
    border-radius: 50%;
    margin-top: 0.25rem;
    background: var(--ink-muted);
}

.row.fault .dot {
    background: var(--status-critical);
}

.row.dimmed .dot {
    opacity: 0.4;
}

.text {
    display: grid;
    gap: 0.0625rem;
}

.name {
    font-size: 0.8125rem;
    font-weight: 600;
    color: var(--ink-primary);
}

.row.fault .name {
    color: var(--status-critical-text);
}

/* After the `.fault` rule deliberately: a dimmed row is no longer happening,
   so it reads as secondary text whatever its tone. */
.row.dimmed .name {
    color: var(--ink-secondary);
}

.tag {
    font-size: 0.7rem;
    font-weight: 500;
    text-transform: uppercase;
    color: var(--ink-muted);
    margin-left: 0.5rem;
}

.description {
    font-size: 0.75rem;
    color: var(--ink-muted);
}
</style>
