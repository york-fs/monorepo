import type { RtdPreventionFlag, TsPreventionFlag } from '@/telemetry'

/**
 * The positive-phrased condition each flag's absence represents — a checklist
 * item reads as ticked when its flag is NOT in the wire array (these are
 * prevention/blocking flags, the inverse of `fuses`'/`online_flags`'
 * "presence is good" convention).
 *
 * Key order is the render order: roughly the sequence these tick off in
 * during a real activation, not the backend's enum order. It lives here
 * rather than in a second array listing every flag again — `Record` already
 * forces each flag to appear exactly once, and JS preserves insertion order
 * for string keys, so this one declaration carries both the labels and the
 * order.
 */
export const TS_PREVENTION_LABELS: Record<TsPreventionFlag, string> = {
    FRONT_OFFLINE: 'Front distribution online',
    PRECHARGE_OFFLINE: 'Precharge online',
    INVERTER_OFFLINE: 'Inverter online',
    PRECHARGE_STATE: 'Precharge ready',
    INVERTER_FAULT: 'Inverter ready',
    BAD_FUSE: 'All fuses OK',
    SHUTDOWN_OPEN: 'Shutdown circuit closed',
    NOT_REQUESTED: 'TS activation requested',
}

export const RTD_PREVENTION_LABELS: Record<RtdPreventionFlag, string> = {
    TS_NOT_ACTIVE: 'TS active',
    BRAKE_NOT_PRESSED: 'Brake pressed',
    NOT_REQUESTED: 'RTD activation requested',
}

export interface PreventionCondition<T extends string> {
    flag: T
    label: string
}

function conditions<T extends string>(labels: Record<T, string>): PreventionCondition<T>[] {
    return (Object.keys(labels) as T[]).map((flag) => ({ flag, label: labels[flag] }))
}

export const TS_PREVENTION_CONDITIONS = conditions(TS_PREVENTION_LABELS)
export const RTD_PREVENTION_CONDITIONS = conditions(RTD_PREVENTION_LABELS)
