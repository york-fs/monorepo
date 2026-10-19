<script setup lang="ts">
import { computed } from 'vue'
import { useSegments } from '@/composables/useSegments'
import { cellTemperatureSeverity, cellVoltageSeverity } from '@/domain/bms'
import { severityOf } from '@/domain/severity'
import { formatCelsius } from '@/utils/formatCelsius'
import { formatVolts } from '@/utils/formatVolts'
import Tile from '@/components/Tile.vue'

// Every reading of one segment under the index the BMS sends it at, so it
// holds whatever the thermistor mapping turns out to be.
const props = defineProps<{ segment: number }>()

const { readings, isStale } = useSegments()

const segment = computed(() => readings.get(props.segment))
const stale = computed(() => isStale(props.segment))

const cellRows = computed(() =>
    (segment.value?.cell_voltages ?? []).map((volts, cell) => ({
        name: `C${cell}`,
        value: formatVolts(volts ?? undefined, 3),
        severity: severityOf(volts, cellVoltageSeverity),
        note: segment.value?.degraded_cells.includes(cell) ? 'degraded' : undefined,
    })),
)
const temperatureRows = computed(() =>
    (segment.value?.temperatures ?? []).map((celsius, index) => ({
        name: `T${index}`,
        value: formatCelsius(celsius ?? undefined),
        severity: severityOf(celsius, cellTemperatureSeverity),
    })),
)
</script>

<template>
    <Tile title="All readings, by BMS index" :level="3">
        <p v-if="segment === undefined" class="empty">Not heard from yet</p>
        <div v-else class="readings" :class="{ stale }">
            <dl class="grid">
                <div
                    v-for="row in cellRows"
                    :key="row.name"
                    class="entry"
                    :data-severity="row.severity"
                >
                    <dt>{{ row.name }}</dt>
                    <dd>
                        {{ row.value }}<span v-if="row.note" class="note"> {{ row.note }}</span>
                    </dd>
                </div>
            </dl>
            <dl class="grid">
                <div
                    v-for="row in temperatureRows"
                    :key="row.name"
                    class="entry"
                    :data-severity="row.severity"
                >
                    <dt>{{ row.name }}</dt>
                    <dd>{{ row.value }}</dd>
                </div>
            </dl>
        </div>
    </Tile>
</template>

<style scoped>
.empty {
    margin: 0;
    font-size: 0.8125rem;
    color: var(--ink-muted);
}

.readings {
    display: grid;
    gap: var(--gap-blocks);
}
.readings.stale {
    opacity: 0.4;
}

.grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(7.5rem, 1fr));
    gap: var(--gap-tight) var(--gap-inline);
    margin: 0;
}

.entry {
    display: flex;
    justify-content: space-between;
    gap: var(--gap-items);
    padding: 0.25rem 0;
    font-size: 0.8125rem;
    border-bottom: 1px solid var(--border);
}

dt {
    color: var(--ink-muted);
}

dd {
    margin: 0;
    font-variant-numeric: tabular-nums;
}
.entry[data-severity='warning'] dd {
    color: var(--status-warning-text);
}
.entry[data-severity='critical'] dd {
    color: var(--status-critical-text);
}

.note {
    color: var(--status-warning-text);
}
</style>
