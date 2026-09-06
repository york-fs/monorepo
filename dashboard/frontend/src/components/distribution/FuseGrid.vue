<script setup lang="ts">
import { computed } from 'vue'
import { FUSE_FLAGS } from '@/domain/fuses'
import { isFuseOk } from '@/telemetry'
import type { FuseFlag } from '@/telemetry'

const props = defineProps<{
    fuses?: readonly FuseFlag[]
}>()

type FuseState = 'unknown' | 'ok' | 'blown'

// The glyph carries the state visually (an intact vs. snapped link wire);
// this is the same information for assistive tech and for a hover tooltip,
// since the visible label underneath is only the flag name.
const FUSE_STATE_LABELS: Record<FuseState, string> = {
    unknown: 'no signal',
    ok: 'OK',
    blown: 'blown',
}

const fuseStates = computed<{ flag: FuseFlag; state: FuseState }[]>(() =>
    FUSE_FLAGS.map((flag) => {
        const ok = isFuseOk(props.fuses, flag)
        const state: FuseState = ok === undefined ? 'unknown' : ok ? 'ok' : 'blown'
        return { flag, state }
    }),
)
</script>

<template>
    <div class="fuses">
        <h3>Fuses</h3>
        <div class="fusebox">
            <div v-for="f in fuseStates" :key="f.flag" class="fuse-cell">
                <div
                    class="ato"
                    role="img"
                    :aria-label="FUSE_STATE_LABELS[f.state]"
                    :title="`${f.flag}: ${FUSE_STATE_LABELS[f.state]}`"
                >
                    <div class="ato-glyph" :data-state="f.state">
                        <span class="ato-highlight"></span>
                        <span v-if="f.state !== 'blown'" class="ato-link"></span>
                        <template v-else>
                            <span class="ato-link-seg left"></span>
                            <span class="ato-link-seg right"></span>
                        </template>
                    </div>
                    <span class="ato-prongs"><span></span><span></span></span>
                </div>
                <span class="fuse-label">{{ f.flag }}</span>
            </div>
        </div>
    </div>
</template>

<style scoped>
.fusebox {
    display: grid;
    /* Explicit column counts rather than auto-fill: there are 17 fuses, and
       17 is prime, so the column count that happens to fit decides whether
       the last row holds a sensible remainder or a single stranded fuse.
       8 gives 8+8+1 and 4 gives 4+4+4+4+1; 9/6/3 give 9+8, 6+6+5 and 3x5+2.
       minmax(0, 1fr) rather than 1fr so a long label can't push a track
       past its share (the labels have no spaces to wrap on — see
       .fuse-label's overflow-wrap). */
    grid-template-columns: repeat(9, minmax(0, 1fr));
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
    /* Belt and braces alongside the track sizing above: a flag name longer
       than its cell wraps rather than overlapping the next fuse. */
    overflow-wrap: anywhere;
}

/* Body + prongs are one unit — .fuse-cell's gap must only separate that
   unit from the label, or it opens a visible gap between the fuse body and
   the prongs that are supposed to be attached to it. */
.ato {
    display: flex;
    flex-direction: column;
    align-items: center;
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
    background: var(--glyph-highlight);
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
    background: var(--glyph-line);
}

.ato-link-seg {
    position: absolute;
    top: 50%;
    width: 22%;
    height: 2px;
    background: var(--glyph-line);
    transform: translateY(-50%) rotate(12deg);
}
.ato-link-seg.left {
    left: 12%;
    transform: translateY(-50%) rotate(-12deg);
}
.ato-link-seg.right {
    right: 12%;
}

@media (max-width: 47.5em) {
    .fusebox {
        grid-template-columns: repeat(6, minmax(0, 1fr));
    }
}

@media (max-width: 30em) {
    .fusebox {
        grid-template-columns: repeat(3, minmax(0, 1fr));
    }
}
</style>
