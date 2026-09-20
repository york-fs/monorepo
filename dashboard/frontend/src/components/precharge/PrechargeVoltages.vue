<script setup lang="ts">
import { formatVolts } from '@/utils/formatVolts'

defineProps<{
    prchgVoltage?: number
    tsVoltage?: number
}>()
</script>

<template>
    <div class="voltages">
        <div class="reading">
            <span class="label">Precharge Rail</span>
            <span class="value">{{ formatVolts(prchgVoltage) }}</span>
        </div>
        <div class="reading">
            <span class="label">Tractive System</span>
            <span class="value">{{ formatVolts(tsVoltage) }}</span>
        </div>
    </div>
</template>

<style scoped>
.voltages {
    display: grid;
    gap: var(--gap-rows);
}

.reading {
    display: grid;
    grid-template-columns: 1fr auto;
    align-items: baseline;
    column-gap: var(--gap-inline);
}

.value {
    font-size: 1.125rem;
    font-weight: 650;
    color: var(--ink-primary);
    text-align: right;
    /* These tick every frame — proportional figures make the digits jitter
       in width as they change. Same rule as AccentTile/MinMaxSub/uptime. */
    font-variant-numeric: tabular-nums;
}

.label {
    font-size: 0.8125rem;
    color: var(--ink-secondary);
}
</style>
