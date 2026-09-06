import type { ShutdownOpenCause } from '@/telemetry'
import type { Severity } from '@/domain/severity'
import { titleCaseEnum } from '@/utils/titleCase'

// Words naive title-casing gets wrong — see titleCaseEnum.
const ACRONYMS: Record<string, string> = {
    BMS: 'BMS',
    IMD: 'IMD',
    TSMS: 'TSMS',
    HVD: 'HVD',
    ESTOP: 'E-Stop',
}

// All three take `undefined` — the field simply hasn't arrived yet — and
// answer for it themselves, so the guard lives once per function rather than
// once per call site. Same convention as the `format*` utils.
export function shutdownOpenCauseLabel(cause: ShutdownOpenCause | undefined): string {
    if (cause === undefined) return '—'
    if (cause === 'NONE') return 'None'
    return titleCaseEnum(cause, ACRONYMS)
}

export function shutdownOpenCauseSeverity(
    cause: ShutdownOpenCause | undefined,
): Severity | undefined {
    if (cause === undefined) return undefined
    return cause === 'NONE' ? 'good' : 'critical'
}

export const SHUTDOWN_OPEN_CAUSE_EXPLANATIONS: Record<ShutdownOpenCause, string> = {
    NONE: 'Shutdown is closed up to the TSAC',
    REAR_INPUT: 'The BSPD or shutdown circuit fuse is open',
    FRONT_ESTOP: 'The front emergency stop button has been pressed',
    BRAKE_OVER_TRAVEL: 'The brake over travel safety switch has activated',
    INERTIA_SWITCH: 'The crash inertia switch has tripped',
    FRONT_AUXILIARY: 'The front auxiliary expansion has opened shutdown',
    FRONT_OUTPUT: 'The shutdown signal is not leaving the front distribution',
    BMS_LATCH: 'The BMS has opened its shutdown hard latch',
    IMD_LATCH: 'The IMD has opened its shutdown hard latch',
    INVERTER_INTERLOCK: "The inverter's interlock is open",
    SHUTDOWN_LATCH_FAILURE: 'The shutdown latch is not operating properly',
    LEFT_ESTOP: 'The rear-left emergency stop button has been pressed',
    RIGHT_ESTOP: 'The rear-right emergency stop button has been pressed',
    HVD_INTERLOCK: "The high voltage disconnect's interlock is open",
    REAR_AUXILIARY: 'The rear auxiliary expansion has opened shutdown',
    TSMS: 'The tractive system master switch is open',
}

export function shutdownOpenCauseExplanation(
    cause: ShutdownOpenCause | undefined,
): string | undefined {
    return cause === undefined ? undefined : SHUTDOWN_OPEN_CAUSE_EXPLANATIONS[cause]
}
