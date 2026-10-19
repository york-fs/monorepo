<script setup lang="ts">
import { onMounted, ref } from 'vue'
import Tile from '@/components/Tile.vue'
import { isDemoMode } from '@/demo'

// One per dashboard server run, recorded by the backend (see sessions.py).
interface Session {
    id: string
    started: string
    bytes: number
    current: boolean
}

const available = !isDemoMode()
const sessions = ref<Session[]>([])
const loadError = ref<string | null>(null)

async function load() {
    try {
        const response = await fetch('/api/sessions')
        if (!response.ok) throw new Error(`HTTP ${response.status}`)
        sessions.value = (await response.json()) as Session[]
        loadError.value = null
    } catch (error) {
        loadError.value = `Couldn't load the sessions (${String(error)})`
    }
}

onMounted(() => {
    if (available) void load()
})

function startedLabel(started: string): string {
    return new Date(started).toLocaleString('en-GB', {
        weekday: 'short',
        day: 'numeric',
        month: 'short',
        hour: '2-digit',
        minute: '2-digit',
    })
}

function sizeLabel(bytes: number): string {
    return bytes < 1e6 ? `${Math.round(bytes / 1e3)} kB` : `${(bytes / 1e6).toFixed(1)} MB`
}
</script>

<template>
    <Tile title="Sessions" :level="4">
        <p v-if="!available" class="note">
            Sessions are recorded by the dashboard server, so there are none in demo mode.
        </p>
        <template v-else>
            <div class="toolbar">
                <p class="note">
                    Each export is a zip of the session's frames, its events and a summary, ready
                    for a script or an AI to analyse.
                </p>
                <button type="button" class="action" @click="load">Refresh</button>
            </div>
            <p v-if="loadError" class="note">{{ loadError }}</p>
            <ol class="list">
                <li v-for="session in sessions" :key="session.id" class="row">
                    <span class="started">{{ startedLabel(session.started) }}</span>
                    <span class="size">{{ sizeLabel(session.bytes) }}</span>
                    <span v-if="session.current" class="current">Recording</span>
                    <a class="action" :href="`/api/sessions/${session.id}/export`" download>
                        Export
                    </a>
                </li>
            </ol>
        </template>
    </Tile>
</template>

<style scoped>
.toolbar {
    display: flex;
    flex-wrap: wrap;
    justify-content: space-between;
    align-items: center;
    gap: var(--gap-rows);
    margin-bottom: 0.75rem;
}

.note {
    margin: 0;
    font-size: 0.8125rem;
    color: var(--ink-muted);
}

.list {
    display: grid;
    gap: var(--gap-tight);
    margin: 0;
    padding: 0;
    list-style: none;
}

.row {
    display: flex;
    align-items: center;
    gap: var(--gap-inline);
    padding: 0.375rem 0;
    font-size: 0.875rem;
    border-top: 1px solid var(--border);
}

.started {
    min-width: 11rem;
    font-variant-numeric: tabular-nums;
}

.size {
    color: var(--ink-muted);
    font-variant-numeric: tabular-nums;
}

.current {
    font-size: 0.75rem;
    color: var(--status-good-text);
}

.action {
    margin-left: auto;
    padding: 0.25rem 0.5rem;
    font: inherit;
    font-size: 0.8125rem;
    color: var(--ink-primary);
    text-decoration: none;
    background: var(--surface-tile);
    border: 1px solid var(--border);
    border-radius: var(--radius-badge);
    cursor: pointer;
}
</style>
