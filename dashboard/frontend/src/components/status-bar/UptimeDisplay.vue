<script setup lang="ts">
import { computed } from 'vue'
import { formatUptime } from '@/utils/formatUptime'

const props = defineProps<{
    ms?: number
    prefix?: string
    text?: string
}>()

const text = computed(
    () =>
        props.text ??
        [props.prefix, formatUptime(props.ms === undefined ? undefined : props.ms / 1000)]
            .filter(Boolean)
            .join(' '),
)
</script>

<template>
    <span class="uptime">{{ text }}</span>
</template>

<style scoped>
.uptime {
    font-size: 0.8125rem;
    color: var(--ink-muted);
    font-variant-numeric: tabular-nums;
}
</style>
