<script setup lang="ts">
import { computed } from 'vue'
import { FUSE_FLAGS } from '@/domain/fuses'
import { isFuseOk } from '@/telemetry'
import type { FuseFlag } from '@/telemetry'

const props = defineProps<{
    fuses?: readonly FuseFlag[]
}>()

type FuseState = 'unknown' | 'ok' | 'blown'

const fuseStates = computed<{ flag: FuseFlag; state: FuseState }[]>(() =>
    FUSE_FLAGS.map((flag) => {
        const ok = isFuseOk(props.fuses, flag)
        const state: FuseState = ok === undefined ? 'unknown' : ok ? 'ok' : 'blown'
        return { flag, state }
    }),
)
</script>

<template>
    <div class="fusebox">
        <div v-for="f in fuseStates" :key="f.flag" class="fuse-cell">
            <div class="ato-glyph" :data-state="f.state">
                <span class="ato-highlight"></span>
                <span v-if="f.state !== 'blown'" class="ato-link"></span>
                <template v-else>
                    <span class="ato-link-seg left"></span>
                    <span class="ato-link-seg right"></span>
                </template>
            </div>
            <span class="ato-prongs"><span></span><span></span></span>
            <span class="fuse-label">{{ f.flag }}</span>
        </div>
    </div>
</template>

<style scoped>
.fusebox {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(5.5rem, 1fr));
    gap: 0.8125rem;
    background: var(--surface-inset);
    border: 1px solid var(--border);
    border-radius: 0.5rem;
    padding: 1.25rem;
}

.fuse-cell {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 0.375rem;
}

.fuse-label {
    font-family: ui-monospace, 'SF Mono', Menlo, Consolas, monospace;
    font-size: 0.6875rem;
    letter-spacing: 0.02em;
    color: var(--ink-secondary);
    text-align: center;
}

/* ATO bodies read wider/flatter than a narrower "Mini" blade shape. */
.ato-glyph {
    width: 2.5rem;
    height: 1.5rem;
    border-radius: 0.1875rem;
    border: 1px solid var(--border);
    display: flex;
    align-items: center;
    justify-content: center;
    position: relative;
    overflow: hidden;
}

.ato-glyph[data-state='ok'] {
    background: var(--status-good);
}
.ato-glyph[data-state='blown'] {
    background: var(--status-critical);
}
.ato-glyph[data-state='unknown'] {
    background: transparent;
}

.ato-highlight {
    position: absolute;
    inset: 0.1875rem auto 0.1875rem 0.25rem;
    width: 0.375rem;
    border-radius: 0.1875rem;
    background: rgba(255, 255, 255, 0.35);
}

.ato-glyph[data-state='unknown'] .ato-highlight {
    display: none;
}

.ato-prongs {
    display: flex;
    justify-content: center;
    gap: 1.25rem;
    height: 0.375rem;
    margin-top: -1px;
}

.ato-prongs span {
    width: 0.1875rem;
    height: 100%;
    background: var(--ink-muted);
}

.ato-link {
    width: 65%;
    height: 2px;
    background: rgba(255, 255, 255, 0.9);
}

.ato-link-seg {
    position: absolute;
    top: 50%;
    width: 22%;
    height: 2px;
    background: rgba(255, 255, 255, 0.9);
    transform: translateY(-50%) rotate(12deg);
}
.ato-link-seg.left {
    left: 12%;
    transform: translateY(-50%) rotate(-12deg);
}
.ato-link-seg.right {
    right: 12%;
}
</style>
