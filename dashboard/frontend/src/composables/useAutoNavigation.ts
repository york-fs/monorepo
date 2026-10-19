import { computed, onUnmounted, ref, watch } from 'vue'
import { useRouter } from 'vue-router'
import { useTelemetry } from '@/composables/useTelemetry'
import type { PageName } from '@/pages'

const STORAGE_KEY = 'yfs-dashboard-auto-switch'

// After you pick a tab yourself, leave you there this long before switching
// for you, so the dash doesn't pull you away from something you're reading.
const MANUAL_HOLD_MS = 30_000

// Storage can be unavailable (private windows, blocked site data), in which
// case the setting still works but isn't remembered across reloads.
function load(): boolean {
    try {
        return localStorage.getItem(STORAGE_KEY) !== 'false'
    } catch {
        return true
    }
}

function save(enabled: boolean) {
    try {
        localStorage.setItem(STORAGE_KEY, String(enabled))
    } catch {
        // Not remembered; see load().
    }
}

// Module-level so the toggle and the navigation share one setting, remembered
// per browser since it's a viewer preference rather than anything about the car.
const autoSwitch = ref(load())
watch(autoSwitch, save)

/** Whether the dash switches pages by itself. */
export function useAutoSwitchSetting() {
    return { autoSwitch }
}

/**
 * Switches to the Drive page when RTD comes on, unless switching is turned
 * off or a tab was picked by hand within the last `MANUAL_HOLD_MS`. Call once,
 * from the root view.
 */
export function useAutoNavigation() {
    const router = useRouter()
    const { frame } = useTelemetry()

    let lastManualAt = -Infinity
    let switching = false

    // The initial load comes from START_LOCATION, which matches nothing, and
    // isn't a choice anyone made.
    const removeHook = router.afterEach((_to, from) => {
        if (!switching && from.matched.length > 0) lastManualAt = Date.now()
    })
    onUnmounted(removeHook)

    async function show(page: PageName) {
        const current = router.currentRoute.value
        if (!autoSwitch.value || current.name === page) return
        if (Date.now() - lastManualAt < MANUAL_HOLD_MS) return

        switching = true
        try {
            await router.push({ name: page, query: current.query })
        } finally {
            switching = false
        }
    }

    // `undefined` until the flags arrive, so the first frame isn't mistaken
    // for RTD coming on.
    const rtdActive = computed(() =>
        frame.rtd_prevention_flags === undefined
            ? undefined
            : frame.rtd_prevention_flags.length === 0,
    )
    watch(rtdActive, (active, before) => {
        if (before === false && active === true) void show('drive')
    })
}
