<script setup lang="ts">
import { useId } from 'vue'
import EventLogSection from '@/components/events/EventLogSection.vue'
import SessionList from '@/components/debug/SessionList.vue'
import LiveFrame from '@/components/debug/LiveFrame.vue'

// Where to go to find out why something happened. Not `Section`s, for the
// same reason as the event log: they dim when the link drops, and this page
// is most useful exactly then.
const headingId = useId()
</script>

<template>
    <EventLogSection />
    <section class="section" :aria-labelledby="headingId">
        <h2 :id="headingId">Data</h2>
        <SessionList />
        <LiveFrame />
    </section>
</template>

<style scoped>
.section {
    display: grid;
    gap: var(--gap-blocks);
}
</style>
