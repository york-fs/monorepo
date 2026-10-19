import { computed, ref, watch } from 'vue'
import { useRoute } from 'vue-router'
import { useTelemetry } from '@/composables/useTelemetry'
import { useEventLog } from '@/composables/useEventLog'
import { FUSE_FLAGS, isFlagOnline, ONLINE_FLAGS } from '@/telemetry'
import type { OnlineFlag } from '@/telemetry'
import { severityOf, worstSeverity, type Severity } from '@/domain/severity'
import type { PageName } from '@/pages'
import { shutdownOpenCauseSeverity } from '@/domain/shutdown'
import { lvVoltageSeverity } from '@/domain/lvs'
import { SEGMENT_LETTERS, segmentErrorSeverity } from '@/domain/segments'
import { isFlagLive, PRECHARGE_FLAG_META } from '@/domain/precharge'
import {
    inverterFaultSeverity,
    inverterTemperatureSeverity,
    motorTemperatureSeverity,
    rpmSeverity,
} from '@/domain/powertrain'
import {
    bmsErrorSeverity,
    cellTemperatureSeverity,
    cellVoltageSeverity,
    packCurrentSeverity,
} from '@/domain/bms'

function onlineSeverity(flags: readonly OnlineFlag[] | undefined, flag: OnlineFlag) {
    const online = isFlagOnline(flags, flag)
    return online === undefined ? undefined : online ? 'good' : 'critical'
}

// Faults logged before the dash was opened are history, not news.
const openedAt = Date.now()

/**
 * Each page's overall state, for tinting its tab: the worst severity of the
 * readings that page shows, using the same rules as their own tiles, so a tab
 * goes red exactly when something on that page does. Car pages go uncoloured
 * while the link is down rather than keep showing a stale verdict.
 *
 * Debug is the exception, reading the event log rather than the car: red once
 * a fault is logged, until the Debug page has been looked at.
 */
export function usePageStatus() {
    const { frame, status } = useTelemetry()
    const { events, available } = useEventLog()
    const route = useRoute()

    const overview = computed(() =>
        worstSeverity(
            frame.online_flags === undefined
                ? undefined
                : ONLINE_FLAGS.every((flag) => frame.online_flags?.includes(flag))
                  ? 'good'
                  : 'critical',
            frame.fuses === undefined
                ? undefined
                : FUSE_FLAGS.every((fuse) => frame.fuses?.includes(fuse))
                  ? 'good'
                  : 'critical',
            shutdownOpenCauseSeverity(frame.shutdown_open_cause),
            severityOf(frame.lvs_min_voltage, lvVoltageSeverity),
            severityOf(frame.min_cell_voltage, cellVoltageSeverity),
            severityOf(frame.max_cell_temperature, cellTemperatureSeverity),
            inverterFaultSeverity(frame.inverter_fault),
            bmsErrorSeverity(frame.bms_master_error_flags),
        ),
    )

    // Not having asked for TS yet is the normal resting state, not a fault.
    const predrive = computed(() =>
        worstSeverity(
            frame.ts_prevention_flags === undefined
                ? undefined
                : frame.ts_prevention_flags.some((flag) => flag !== 'NOT_REQUESTED')
                  ? 'critical'
                  : 'good',
            frame.precharge_error_flags === undefined
                ? undefined
                : frame.precharge_error_flags.some(
                        (flag) =>
                            PRECHARGE_FLAG_META[flag].kind === 'fault' &&
                            isFlagLive(flag, frame.precharge_state),
                    )
                  ? 'critical'
                  : 'good',
        ),
    )

    const drive = computed(() =>
        worstSeverity(
            onlineSeverity(frame.online_flags, 'INVERTER_ONLINE'),
            inverterFaultSeverity(frame.inverter_fault),
            severityOf(frame.motor_rpm, rpmSeverity),
            severityOf(frame.motor_temperature, motorTemperatureSeverity),
            severityOf(frame.inverter_temperature, inverterTemperatureSeverity),
        ),
    )

    const bms = computed(() =>
        worstSeverity(
            onlineSeverity(frame.online_flags, 'BMS_ONLINE'),
            bmsErrorSeverity(frame.bms_master_error_flags),
            severityOf(frame.min_cell_voltage, cellVoltageSeverity),
            severityOf(frame.max_cell_voltage, cellVoltageSeverity),
            severityOf(frame.max_cell_temperature, cellTemperatureSeverity),
            severityOf(frame.positive_current, packCurrentSeverity),
            ...SEGMENT_LETTERS.map((_, id) =>
                segmentErrorSeverity(frame.segment_error_flags?.[id]),
            ),
        ),
    )

    const seenUpTo = ref(0)
    const newestId = computed(() => events.value.at(-1)?.id ?? 0)
    watch(
        [() => route.name, newestId],
        ([page, newest]) => {
            if (page === 'debug') seenUpTo.value = newest
        },
        { immediate: true },
    )

    const debug = computed<Severity | undefined>(() => {
        if (!available.value) return undefined
        const unseenFault = events.value.some(
            (event) =>
                event.id > seenUpTo.value && event.time >= openedAt && event.severity === 'fault',
        )
        return unseenFault ? 'critical' : 'good'
    })

    return computed<Record<PageName, Severity | undefined>>(() => {
        const live = status.value === 'online'
        return {
            overview: live ? overview.value : undefined,
            predrive: live ? predrive.value : undefined,
            drive: live ? drive.value : undefined,
            bms: live ? bms.value : undefined,
            debug: debug.value,
        }
    })
}
