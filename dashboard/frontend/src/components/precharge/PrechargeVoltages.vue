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
    gap: 0.5rem;
}

.reading {
    display: grid;
    grid-template-columns: 1fr auto;
    align-items: baseline;
    column-gap: 0.75rem;
}

.value {
    font-size: 1.125rem;
    font-weight: 650;
    color: var(--ink-primary);
    text-align: right;
}

.label {
    font-size: 0.8125rem;
    color: var(--ink-secondary);
}
</style>
