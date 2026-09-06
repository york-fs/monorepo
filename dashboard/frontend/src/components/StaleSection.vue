<script setup lang="ts">
import { useId } from 'vue'
import { useComponentStatus } from '@/composables/useComponentStatus'

const props = withDefaults(
    defineProps<{
        /** Section heading — rendered as the `h2` that names the region. */
        title: string
        /** This section's own online signal, if it has one — see `useComponentStatus`. */
        online?: boolean
    }>(),
    { online: undefined },
)

const { status } = useComponentStatus(() => props.online)

// The `<section>` needs an accessible name to be exposed as a region, and a
// heading inside it doesn't provide one on its own — hence pointing
// aria-labelledby at the h2 rather than relying on document structure.
const headingId = useId()
</script>

<template>
    <section class="stale-section" :aria-labelledby="headingId">
        <div class="header-row">
            <h2 :id="headingId">{{ title }}</h2>
            <!-- Always present, so assistive tech has a live region to
                 announce into when the banner appears; a `role="status"` that
                 only enters the DOM alongside its own text is unreliable.
                 Collapses via `:empty` so it doesn't leave a permanent gap
                 after the heading (a comment node still counts as empty). -->
            <div class="banner-slot" role="status">
                <!-- No "last seen Ns ago" here — that duration is the whole
                     telemetry link's freshness (see useLastSeen), not specific
                     to this section's own online signal, so showing it here
                     would misrepresent how long *this* section has been
                     stale. -->
                <span v-if="status === 'offline'" class="stale-banner">Stale</span>
            </div>
        </div>
        <div class="content" :class="{ dimmed: status === 'offline' }">
            <slot />
        </div>
    </section>
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

.banner-slot {
    display: flex;
}
.banner-slot:empty {
    display: none;
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

/* Every section lays its blocks out the same way, so this lives here rather
   than being redeclared by each one's own wrapper element. */
.content {
    display: grid;
    gap: 1.25rem;
}

.content.dimmed {
    opacity: 0.5;
}
</style>
