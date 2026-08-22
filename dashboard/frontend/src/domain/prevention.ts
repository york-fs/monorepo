import type { RtdPreventionFlag, TsPreventionFlag } from '@/telemetry'

/**
 * Render order + the positive-phrased condition each flag's absence
 * represents — a checklist item reads as ticked when its flag is NOT in the
 * wire array (these are prevention/blocking flags, the inverse of
 * `fuses`'/`online_flags`' "presence is good" convention). Ordered roughly
 * by the sequence these tick off in during a real activation, not enum
 * declaration order.
 */
export const TS_PREVENTION_FLAGS: TsPreventionFlag[] = [
    'FRONT_OFFLINE',
    'PRECHARGE_OFFLINE',
    'PRECHARGE_STATE',
    'BAD_FUSE',
    'SHUTDOWN_OPEN',
    'NOT_REQUESTED',
]

export const TS_PREVENTION_LABELS: Record<TsPreventionFlag, string> = {
    SHUTDOWN_OPEN: 'Shutdown circuit closed',
    BAD_FUSE: 'All fuses OK',
    FRONT_OFFLINE: 'Front distribution online',
    NOT_REQUESTED: 'TS activation requested',
    PRECHARGE_OFFLINE: 'Precharge online',
    PRECHARGE_STATE: 'Precharge ready',
}

export const RTD_PREVENTION_FLAGS: RtdPreventionFlag[] = [
    'TS_NOT_ACTIVE',
    'BRAKE_NOT_PRESSED',
    'NOT_REQUESTED',
]

export const RTD_PREVENTION_LABELS: Record<RtdPreventionFlag, string> = {
    TS_NOT_ACTIVE: 'TS active',
    NOT_REQUESTED: 'RTD activation requested',
    BRAKE_NOT_PRESSED: 'Brake pressed',
}
