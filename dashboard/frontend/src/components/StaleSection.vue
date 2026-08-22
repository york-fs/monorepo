<script setup lang="ts">
import { useComponentStatus } from '@/composables/useComponentStatus'

const props = withDefaults(
    defineProps<{
        /** This section's own online signal, if it has one — see `useComponentStatus`. */
        online?: boolean
    }>(),
    { online: undefined },
)

const { status } = useComponentStatus(() => props.online)
</script>

<template>
    <div class="stale-section">
        <div class="header-row">
            <slot name="header" />
            <!-- No "last seen Ns ago" here — that duration is the whole
                 telemetry link's freshness (see useLastSeen), not specific to
                 this section's own online signal, so showing it here would
                 misrepresent how long *this* section has actually been stale. -->
            <span v-if="status === 'offline'" class="stale-banner">Stale</span>
        </div>
        <div class="content" :class="{ dimmed: status === 'offline' }">
            <slot />
        </div>
    </div>
</template>

<style scoped>
.stale-section {
    display: grid;
    gap: 1.25rem;
}

.header-row {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 0.75rem;
}

.stale-banner {
    background: color-mix(in srgb, var(--status-warning) 16%, var(--surface-card));
    color: var(--status-warning-text);
    border: 1px solid var(--border);
    border-radius: 0.25rem;
    font-size: 0.75rem;
    font-weight: 600;
    padding: 0.125rem 0.5rem;
}

.content.dimmed {
    opacity: 0.5;
}
</style>
