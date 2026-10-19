<script setup lang="ts">
import { computed, onMounted, useTemplateRef, watch } from 'vue'

// Shows one of the hand-drawn SVGs in assets/ and writes live values into it
// by selector, so the drawing stays the source of truth for the layout and
// can be re-exported without touching any code. Its own <style> is dropped:
// the parent themes it with `:deep()` rules instead, and it can't leak into
// the rest of the page.

export interface SvgBinding {
    /** Every element matching this gets the text and attributes below. */
    selector: string
    text?: string
    /** `undefined` removes the attribute. */
    attrs?: Record<string, string | undefined>
}

const props = defineProps<{
    svg: string
    bindings: SvgBinding[]
}>()

const markup = computed(() => props.svg.replace(/<style>[\s\S]*?<\/style>/, ''))
const root = useTemplateRef<HTMLDivElement>('root')

function apply() {
    if (!root.value) return
    for (const { selector, text, attrs } of props.bindings) {
        for (const element of root.value.querySelectorAll(selector)) {
            if (text !== undefined) element.textContent = text
            for (const [name, value] of Object.entries(attrs ?? {})) {
                if (value === undefined) element.removeAttribute(name)
                else element.setAttribute(name, value)
            }
        }
    }
}

onMounted(apply)
watch([markup, () => props.bindings], apply, { flush: 'post' })
</script>

<template>
    <!-- eslint-disable-next-line vue/no-v-html -- our own bundled asset, never user input -->
    <div ref="root" class="drawing" v-html="markup" />
</template>

<style scoped>
.drawing :deep(svg) {
    display: block;
    width: 100%;
    height: auto;
}

.drawing :deep(text) {
    font-family: var(--font-sans);
    fill: var(--ink-primary);
}
</style>
