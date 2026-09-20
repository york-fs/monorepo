<script setup lang="ts">
// A named block within a section: the `.subheading` tier (see style.css) plus
// whatever it names. The middle rung of the heading hierarchy, between a
// section's h2 and the quiet `.tile-label` on each individual tile.
//
// Renders an h3 rather than a bare styled element so the grouping is real
// structure and not just a visual one — the tiles inside it then carry h4
// labels, keeping heading order honest for anything navigating by headings.
defineProps<{ title: string }>()
</script>

<template>
    <div>
        <h3 class="subheading">{{ title }}</h3>
        <slot />
    </div>
</template>
