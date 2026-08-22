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
    LED_CHECK: 'LED check',
    PRECHECK: 'Precheck',
    STANDBY: 'Standby',
    PRECHARGE: 'Precharging',
    PRECHARGE_HOLD: 'Precharge hold',
    ACTIVE: 'Active',
}

type PrechargeFlagKind = 'fault' | 'waiting' | 'deactivation'

interface PrechargeFlagMeta {
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
        description: 'Discharge relay measured open when it should have been closed',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY'],
    },
    PRECHARGE_CLOSED: {
        description: 'Precharge relay measured closed when it should have been open',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY', 'ACTIVE'],
    },
    AIR_POS_CLOSED: {
        description: 'Main positive relay measured closed when it should have been open',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY', 'PRECHARGE'],
    },
    AIR_NEG_CLOSED: {
        description: 'Main negative relay measured closed when it should have been open',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY'],
    },
    PRECHECK_VOLTAGE: {
        description: 'Unexpected voltage measured on the output side of the precharge relay',
        kind: 'fault',
        liveStates: ['PRECHECK', 'STANDBY'],
    },
    WAITING_DISCHARGE: {
        description: 'TS-side voltage needs to drain before proceeding',
        kind: 'waiting',
        liveStates: ['PRECHECK', 'STANDBY'],
    },
    WAITING_ACTIVATION: {
        description: 'Awaiting an activation request over CAN',
        kind: 'waiting',
        liveStates: ['STANDBY'],
    },
    SHUTDOWN_OPEN: {
        description: 'Shutdown circuit measured open',
        kind: 'fault',
        liveStates: ['PRECHARGE', 'PRECHARGE_HOLD', 'ACTIVE'],
    },
    PRECHARGE_OPEN: {
        description: 'Precharge relay measured open when it should have been closed',
        kind: 'fault',
        liveStates: ['PRECHARGE', 'PRECHARGE_HOLD'],
    },
    AIR_POS_OPEN: {
        description: 'Main positive relay measured open when it should have been closed',
        kind: 'fault',
        liveStates: ['PRECHARGE_HOLD', 'ACTIVE'],
    },
    AIR_NEG_OPEN: {
        description: 'Main negative relay measured open when it should have been closed',
        kind: 'fault',
        liveStates: ['PRECHARGE', 'PRECHARGE_HOLD', 'ACTIVE'],
    },
    DEACTIVATION: {
        description: 'High voltage deactivated via a CAN request',
        kind: 'deactivation',
        liveStates: ['ACTIVE'],
    },
    DEVIATION: {
        description: "TS voltage didn't match the expected precharge RC curve",
        kind: 'fault',
        liveStates: ['PRECHARGE'],
    },
}

export const PRECHARGE_RELAY_LABELS: Record<PrechargeRelay, string> = {
    DISCHARGE_CLOSED: 'Discharge',
    PRECHARGE_CLOSED: 'Precharge',
    AIR_POS_CLOSED: 'AIR +',
    AIR_NEG_CLOSED: 'AIR −',
}

export function isFlagLive(
    flag: PrechargeErrorFlag,
    currentState: PrechargeState | undefined,
): boolean {
    if (!currentState) return false
    return PRECHARGE_FLAG_META[flag].liveStates.includes(currentState)
}
