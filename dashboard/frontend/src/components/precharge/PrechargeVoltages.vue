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
            <span class="value">{{ formatVolts(prchgVoltage) }}</span>
            <span class="label">Precharge rail</span>
        </div>
        <div class="reading">
            <span class="value">{{ formatVolts(tsVoltage) }}</span>
            <span class="label">Tractive system</span>
        </div>
    </div>
</template>

<style scoped>
.voltages {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 1rem;
}

.reading {
    display: grid;
    gap: 0.125rem;
}

.value {
    font-size: 1.5rem;
    font-weight: 650;
    color: var(--ink-primary);
}

.label {
    font-size: 0.6875rem;
    color: var(--ink-muted);
    text-transform: uppercase;
    letter-spacing: 0.04em;
}
</style>
