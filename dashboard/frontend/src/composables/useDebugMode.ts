import { ref, watch } from 'vue'

const STORAGE_KEY = 'yfs-dashboard-debug'

// Storage can be unavailable (private windows, blocked site data), in which
// case debug mode still works but isn't remembered across reloads.
function load(): boolean {
    try {
        return localStorage.getItem(STORAGE_KEY) === 'true'
    } catch {
        return false
    }
}

function save(enabled: boolean) {
    try {
        localStorage.setItem(STORAGE_KEY, String(enabled))
    } catch {
        // Not remembered; see load().
    }
}

// Module-level singleton so the toggle and everything it reveals share one
// state, remembered per browser since it's a viewer preference rather than
// anything about the car.
const debugMode = ref(load())
watch(debugMode, save)

/**
 * Whether the dashboard shows its debugging tools — the event log, with the
 * last TS/RTD loss tiles and the full history.
 */
export function useDebugMode() {
    return { debugMode }
}
