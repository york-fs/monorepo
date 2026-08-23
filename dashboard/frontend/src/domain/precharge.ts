import type { PrechargeErrorFlag, PrechargeRelay, PrechargeState } from '@/telemetry'

export const PRECHARGE_STATES: PrechargeState[] = [
    'LED_CHECK',
    'PRECHECK',
    'STANDBY',
    'PRECHARGE',
    'PRECHARGE_HOLD',
    'ACTIVE',
]

export const PRECHARGE_STATE_LABELS: Record<PrechargeState, string> = {
    LED_CHECK: 'LED Check',
    PRECHECK: 'Precheck',
    STANDBY: 'Standby',
    PRECHARGE: 'Precharge',
    PRECHARGE_HOLD: 'Precharge Hold',
    ACTIVE: 'Active',
}

type PrechargeFlagKind = 'fault' | 'waiting' | 'deactivation'

interface PrechargeFlagMeta {
    label: string
    description: string
    kind: PrechargeFlagKind
    /**
     * States in which this flag is actively re-evaluated. Latched flags only
     * clear when a new precharge cycle is attempted, so a flag showing up
     * while the current state isn't in this list means "from the last
     * attempt", not "happening right now".
     */
    liveStates: PrechargeState[]
}

export const PRECHARGE_FLAG_META: Record<PrechargeErrorFlag, PrechargeFlagMeta> = {
    DISCHARGE_OPEN: {
        label: 'Discharge Open',
        description: 'Discharge relay measured open when it should have been closed',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY'],
    },
    PRECHARGE_CLOSED: {
        label: 'Precharge Closed',
        description: 'Precharge relay measured closed when it should have been open',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY', 'ACTIVE'],
    },
    AIR_POS_CLOSED: {
        label: 'AIR+ Closed',
        description: 'Positive AIR measured closed when it should have been open',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY', 'PRECHARGE'],
    },
    AIR_NEG_CLOSED: {
        label: 'AIR- Closed',
        description: 'Negative AIR measured closed when it should have been open',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY'],
    },
    PRECHECK_VOLTAGE: {
        label: 'Precheck Voltage',
        description: 'Unexpected voltage measured on the output side of the precharge relay',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY'],
    },
    WAITING_DISCHARGE: {
        label: 'Waiting Discharge',
        description: 'Waiting for TS-side voltage to drain before proceeding',
        kind: 'waiting',
        liveStates: ['PRECHECK', 'STANDBY'],
    },
    WAITING_ACTIVATION: {
        label: 'Waiting Activation',
        description: 'Awaiting an activation request over CAN',
        kind: 'waiting',
        liveStates: ['STANDBY'],
    },
    SHUTDOWN_OPEN: {
        label: 'Shutdown Open',
        description: 'Shutdown circuit measured open',
        kind: 'fault',
        liveStates: ['PRECHARGE', 'PRECHARGE_HOLD', 'ACTIVE'],
    },
    PRECHARGE_OPEN: {
        label: 'Precharge Open',
        description: 'Precharge relay measured open when it should have been closed',
        kind: 'fault',
        liveStates: ['PRECHARGE', 'PRECHARGE_HOLD'],
    },
    AIR_POS_OPEN: {
        label: 'AIR+ Open',
        description: 'Positive AIR measured open when it should have been closed',
        kind: 'fault',
        liveStates: ['PRECHARGE_HOLD', 'ACTIVE'],
    },
    AIR_NEG_OPEN: {
        label: 'AIR- Open',
        description: 'Negative AIR measured open when it should have been closed',
        kind: 'fault',
        liveStates: ['PRECHARGE', 'PRECHARGE_HOLD', 'ACTIVE'],
    },
    DEACTIVATION: {
        label: 'Deactivation',
        description: 'High voltage deactivated via a CAN request',
        kind: 'deactivation',
        liveStates: ['PRECHARGE', 'PRECHARGE_HOLD', 'ACTIVE'],
    },
    DEVIATION: {
        label: 'Deviation',
        description: "TS voltage didn't match the expected precharge RC curve",
        kind: 'fault',
        liveStates: ['PRECHARGE'],
    },
}

export const PRECHARGE_RELAY_LABELS: Record<PrechargeRelay, string> = {
    DISCHARGE_CLOSED: 'Discharge',
    PRECHARGE_CLOSED: 'Precharge',
    AIR_POS_CLOSED: 'AIR+',
    AIR_NEG_CLOSED: 'AIR−',
}

export function isFlagLive(
    flag: PrechargeErrorFlag,
    currentState: PrechargeState | undefined,
): boolean {
    if (!currentState) return false
    return PRECHARGE_FLAG_META[flag].liveStates.includes(currentState)
}
