<script setup lang="ts">
// The explanatory-subtext shape for an AccentTile's `sub` slot — counterpart
// to MinMaxSub.vue's arrow pair, and extracted for the same reason: both
// ShutdownCauseTile and InverterFaultTile had an identical copy of it. Those
// two stay separate components (their severity vocabularies are about to
// diverge — see plan/POWERTRAIN.md); it's only this shared subtext styling
// that had no business being duplicated.
defineProps<{
    text: string
}>()
</script>

<template>
    <span class="explanation">{{ text }}</span>
</template>

<style scoped>
.explanation {
    font-size: 0.8125rem;
    color: var(--ink-muted);
}
</style>
