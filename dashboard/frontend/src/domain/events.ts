import {
    FUSE_FLAGS,
    ONLINE_FLAGS,
    type BmsMasterErrorFlag,
    type FuseFlag,
    type InverterFaultCode,
    type OnlineFlag,
    type PrechargeErrorFlag,
    type PrechargeRelay,
    type PrechargeState,
    type RtdPreventionFlag,
    type ShutdownOpenCause,
    type TsPreventionFlag,
} from '@/telemetry'
import { BMS_ERROR_META } from '@/domain/bms'
import { INVERTER_FAULT_EXPLANATIONS, inverterFaultLabel } from '@/domain/powertrain'
import { PRECHARGE_FLAG_META, PRECHARGE_STATE_LABELS } from '@/domain/precharge'
import { RTD_PREVENTION_LABELS, TS_PREVENTION_LABELS } from '@/domain/prevention'
import { SHUTDOWN_OPEN_CAUSE_EXPLANATIONS, shutdownOpenCauseLabel } from '@/domain/shutdown'
import { titleCaseEnum } from '@/utils/titleCase'

/**
 * The event log is built by the backend (`dashboard/events.py`), which
 * compares consecutive frames and records every change — it keeps running and
 * persisting to disk whether or not a browser is open. This file only knows
 * how to describe those events; the vocabulary (`category`, `kind`, `code`)
 * is the backend's.
 */
export const EVENT_CATEGORIES = [
    'shutdown',
    'ts',
    'rtd',
    'bms',
    'precharge',
    'inverter',
    'online',
    'fuse',
    'system',
] as const
export type EventCategory = (typeof EVENT_CATEGORIES)[number]

export const EVENT_CATEGORY_LABELS: Record<EventCategory, string> = {
    shutdown: 'Shutdown',
    ts: 'TS',
    rtd: 'RTD',
    bms: 'BMS',
    precharge: 'Precharge',
    inverter: 'Inverter',
    online: 'Connection',
    fuse: 'Fuse',
    system: 'Telemetry',
}

export type EventKind =
    | 'raised'
    /** Set and cleared again between two radio frames. */
    | 'blip'
    | 'cleared'
    | 'state'
    | 'activated'
    | 'deactivated'
    | 'aborted'
    | 'connected'
    | 'gap'
    | 'reboot'

/**
 * `fault` — something went wrong; `ok` — a problem cleared or something
 * activated; `info` — routine progress (waiting on the driver, a state change)
 * that's hidden by default.
 */
export type EventSeverity = 'fault' | 'ok' | 'info'

/** The subset of a frame the backend captures alongside deactivations. */
export interface SnapshotFrame {
    shutdown_open_cause?: ShutdownOpenCause
    ts_prevention_flags?: TsPreventionFlag[]
    rtd_prevention_flags?: RtdPreventionFlag[]
    online_flags?: OnlineFlag[]
    fuses?: FuseFlag[]
    precharge_state?: PrechargeState
    precharge_error_flags?: PrechargeErrorFlag[]
    precharge_relay_states?: PrechargeRelay[]
    precharge_prchg_voltage?: number
    precharge_ts_voltage?: number
    bms_master_error_flags?: BmsMasterErrorFlag[]
    bms_i2c_error_count?: number
    inverter_fault?: InverterFaultCode
    inverter_input_voltage?: number
    motor_current?: number
    motor_rpm?: number
    positive_current?: number
    negative_current?: number
    min_cell_voltage?: number
    max_cell_voltage?: number
    max_cell_temperature?: number
    lvs_min_voltage?: number
    // Everything seen since the previous frame — only sent by newer rear
    // firmware, which samples at 100 Hz but transmits at 10 Hz.
    ts_prevention_flags_seen?: TsPreventionFlag[]
    rtd_prevention_flags_seen?: RtdPreventionFlag[]
    shutdown_open_causes_seen?: ShutdownOpenCause[]
    blown_fuses_seen?: FuseFlag[]
    expander_error_count?: number
}

export interface LeadUpEntry {
    category: EventCategory
    code: string
    severity: EventSeverity
    /** How long before the snapshotted event this was raised. */
    before_ms: number
}

export interface EventSnapshot {
    frame: SnapshotFrame
    /** Problems raised in the couple of seconds before, newest first. */
    lead_up: LeadUpEntry[]
}

export interface LogEvent {
    id: number
    /** Wall-clock time the frame was received, in ms since the epoch. */
    time: number
    /** Rear distribution uptime in ms at the time. */
    uptime?: number
    category: EventCategory
    kind: EventKind
    severity: EventSeverity
    code?: string
    previous?: string
    detail?: string
    snapshot?: EventSnapshot
}

const ACRONYMS: Record<string, string> = {
    APPS: 'APPS',
    BMS: 'BMS',
    DWIN: 'DWIN',
    IMD: 'IMD',
    RTD: 'RTD',
    TS: 'TS',
    TSAC: 'TSAC',
    TSAL: 'TSAL',
    LED: 'LED',
}

/**
 * The backend's code for failed rear GPIO expander reads, logged under
 * `shutdown` because a failed read makes every rear shutdown input read open.
 */
const EXPANDER_READ_CODE = 'EXPANDER_READ'

function onlineName(code: string): string {
    return titleCaseEnum(code.replace(/_ONLINE$/, ''), ACRONYMS)
}

// Each lookup falls back to plain title-casing, so a flag added to the
// firmware later still shows up readably rather than crashing the list.
export function codeLabel(category: EventCategory, code: string): string {
    if (code === EXPANDER_READ_CODE) return 'Shutdown input read failed'
    switch (category) {
        case 'shutdown':
            return shutdownOpenCauseLabel(code as ShutdownOpenCause)
        case 'inverter':
            return inverterFaultLabel(code as InverterFaultCode)
        case 'bms':
            return BMS_ERROR_META[code as BmsMasterErrorFlag]?.label ?? titleCaseEnum(code)
        case 'precharge':
            return PRECHARGE_FLAG_META[code as PrechargeErrorFlag]?.label ?? titleCaseEnum(code)
        case 'online':
            return `${onlineName(code)} offline`
        case 'fuse':
            return `${titleCaseEnum(code, ACRONYMS)} fuse blown`
        default:
            return titleCaseEnum(code, ACRONYMS)
    }
}

export function codeDescription(category: EventCategory, code: string): string | undefined {
    if (code === EXPANDER_READ_CODE) {
        // A failed read makes every rear shutdown input read as open for that
        // control cycle, which the rear's TS grace period normally rides out.
        return "The rear GPIO expander's I2C read failed, so the rear shutdown inputs read as open for one cycle. The TS grace period normally rides this out"
    }
    switch (category) {
        case 'shutdown':
            return SHUTDOWN_OPEN_CAUSE_EXPLANATIONS[code as ShutdownOpenCause]
        case 'inverter':
            return INVERTER_FAULT_EXPLANATIONS[code as InverterFaultCode]
        case 'bms':
            return BMS_ERROR_META[code as BmsMasterErrorFlag]?.description
        case 'precharge':
            return PRECHARGE_FLAG_META[code as PrechargeErrorFlag]?.description
        case 'ts': {
            const condition = TS_PREVENTION_LABELS[code as TsPreventionFlag]
            return condition && `TS needs: ${condition}`
        }
        case 'rtd': {
            const condition = RTD_PREVENTION_LABELS[code as RtdPreventionFlag]
            return condition && `RTD needs: ${condition}`
        }
        case 'online':
            return 'The rear distribution stopped receiving its CAN messages'
        case 'fuse':
            return 'Fuse output is more than 2 V below the highest LV fuse voltage'
        default:
            return undefined
    }
}

function stateLabel(state: string | undefined): string {
    return (state && PRECHARGE_STATE_LABELS[state as PrechargeState]) ?? state ?? '—'
}

export function eventTitle(event: LogEvent): string {
    const code = event.code ?? ''
    switch (event.kind) {
        case 'raised':
            return codeLabel(event.category, code)
        case 'blip':
            if (code === EXPANDER_READ_CODE)
                return `${codeLabel(event.category, code)} ×${event.detail}`
            if (event.category === 'fuse') return `${titleCaseEnum(code, ACRONYMS)} fuse dipped`
            return `${codeLabel(event.category, code)} (brief)`
        case 'cleared':
            if (event.category === 'online') return `${onlineName(code)} back online`
            if (event.category === 'fuse') return `${titleCaseEnum(code, ACRONYMS)} fuse OK`
            return `${codeLabel(event.category, code)} cleared`
        case 'state':
            return `Precharge ${stateLabel(event.previous)} → ${stateLabel(code)}`
        case 'activated':
            return event.category === 'ts' ? 'TS activated' : 'Ready to drive'
        case 'deactivated':
            return event.category === 'ts' ? 'TS deactivated' : 'Ready to drive lost'
        case 'aborted':
            return 'Precharge aborted'
        case 'connected':
            return 'Telemetry connected'
        case 'gap':
            return `No telemetry for ${event.detail ?? '?'} s`
        case 'reboot':
            return 'Rear distribution rebooted'
    }
}

export function eventDescription(event: LogEvent): string | undefined {
    if (event.kind === 'blip' && event.code) {
        const description = codeDescription(event.category, event.code)
        const brief = 'Set and cleared between radio frames, so under 100 ms'
        return description ? `${brief}. ${description}` : brief
    }
    if ((event.kind === 'raised' || event.kind === 'cleared') && event.code) {
        return codeDescription(event.category, event.code)
    }
    switch (event.kind) {
        case 'deactivated':
        case 'aborted':
            return 'See what was set at the time and the lead-up below'
        case 'connected':
            return 'Started receiving frames — the state at the time is below'
        case 'gap':
            return 'Changes during the gap are logged at the time of the next frame'
        case 'reboot':
            return 'Uptime went backwards, so the car was power cycled or reset'
        default:
            return undefined
    }
}

export interface SnapshotGroup {
    label: string
    items: string[]
}

/** Everything that was wrong at the time of a snapshot, grouped by subsystem. */
export function snapshotProblems(frame: SnapshotFrame): SnapshotGroup[] {
    const groups: SnapshotGroup[] = [
        {
            label: 'Shutdown open',
            items:
                frame.shutdown_open_cause && frame.shutdown_open_cause !== 'NONE'
                    ? [shutdownOpenCauseLabel(frame.shutdown_open_cause)]
                    : [],
        },
        {
            label: 'Blocking TS',
            items: (frame.ts_prevention_flags ?? []).map((code) => codeLabel('ts', code)),
        },
        {
            label: 'Blocking RTD',
            items: (frame.rtd_prevention_flags ?? []).map((code) => codeLabel('rtd', code)),
        },
        {
            label: 'BMS errors',
            items: (frame.bms_master_error_flags ?? []).map((code) => codeLabel('bms', code)),
        },
        {
            label: `Precharge flags (${stateLabel(frame.precharge_state)})`,
            items: (frame.precharge_error_flags ?? []).map((code) => codeLabel('precharge', code)),
        },
        {
            label: 'Inverter fault',
            items:
                frame.inverter_fault && frame.inverter_fault !== 'NONE'
                    ? [inverterFaultLabel(frame.inverter_fault)]
                    : [],
        },
        {
            label: 'Offline',
            items: frame.online_flags
                ? ONLINE_FLAGS.filter((flag) => !frame.online_flags!.includes(flag)).map(onlineName)
                : [],
        },
        {
            label: 'Blown fuses',
            items: frame.fuses
                ? FUSE_FLAGS.filter((flag) => !frame.fuses!.includes(flag)).map((flag) =>
                      titleCaseEnum(flag, ACRONYMS),
                  )
                : [],
        },
        {
            // Set at some point since the previous frame but not any more — the
            // likeliest culprits for a deactivation nothing else explains.
            label: 'Seen briefly',
            items: [
                ...(frame.shutdown_open_causes_seen ?? [])
                    .filter((code) => code !== frame.shutdown_open_cause)
                    .map((code) => `Shutdown ${shutdownOpenCauseLabel(code)}`),
                ...(frame.ts_prevention_flags_seen ?? [])
                    .filter((code) => !frame.ts_prevention_flags?.includes(code))
                    .map((code) => `TS ${codeLabel('ts', code)}`),
                ...(frame.rtd_prevention_flags_seen ?? [])
                    .filter((code) => !frame.rtd_prevention_flags?.includes(code))
                    .map((code) => `RTD ${codeLabel('rtd', code)}`),
                ...(frame.blown_fuses_seen ?? [])
                    .filter((code) => frame.fuses?.includes(code))
                    .map((code) => `${titleCaseEnum(code, ACRONYMS)} fuse dipped`),
            ],
        },
    ]
    return groups.filter((group) => group.items.length > 0)
}

export interface SnapshotReading {
    label: string
    value: string
}

function reading(label: string, value: number | undefined, unit: string, digits = 0) {
    return value === undefined ? undefined : { label, value: `${value.toFixed(digits)} ${unit}` }
}

/** Key measurements at the time of a snapshot. */
export function snapshotReadings(frame: SnapshotFrame): SnapshotReading[] {
    return [
        reading('TS voltage', frame.precharge_ts_voltage, 'V'),
        // Measured after the precharge relay, so it reads 0 V once that relay
        // opens in the active state.
        reading('Precharge sample', frame.precharge_prchg_voltage, 'V'),
        reading('Inverter input', frame.inverter_input_voltage, 'V'),
        reading('Motor current', frame.motor_current, 'A', 1),
        reading('Pack current (+)', frame.positive_current, 'A', 1),
        reading('Min cell', frame.min_cell_voltage, 'V', 3),
        reading('Max cell temp', frame.max_cell_temperature, '°C'),
        reading('LV min', frame.lvs_min_voltage, 'V', 2),
        reading('I2C errors', frame.bms_i2c_error_count, ''),
        reading('Expander read failures', frame.expander_error_count, ''),
    ].filter((r): r is SnapshotReading => r !== undefined)
}

export function formatEventTime(time: number): string {
    const date = new Date(time)
    const clock = date.toLocaleTimeString([], { hour12: false })
    const tenths = Math.floor(date.getMilliseconds() / 100)
    const today = new Date().toDateString() === date.toDateString()
    const day = today ? '' : `${date.toLocaleDateString([], { day: 'numeric', month: 'short' })} `
    return `${day}${clock}.${tenths}`
}
