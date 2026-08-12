export type PrechargeState =
    'LED_CHECK' | 'PRECHECK' | 'STANDBY' | 'PRECHARGE' | 'PRECHARGE_HOLD' | 'ACTIVE'

export type PrechargeErrorFlag =
    | 'DISCHARGE_OPEN'
    | 'PRECHARGE_CLOSED'
    | 'AIR_POS_CLOSED'
    | 'AIR_NEG_CLOSED'
    | 'PRECHECK_VOLTAGE'
    | 'WAITING_DISCHARGE'
    | 'WAITING_ACTIVATION'
    | 'SHUTDOWN_OPEN'
    | 'PRECHARGE_OPEN'
    | 'AIR_POS_OPEN'
    | 'AIR_NEG_OPEN'
    | 'DEACTIVATION'
    | 'DEVIATION'

export type PrechargeRelay =
    'DISCHARGE_CLOSED' | 'PRECHARGE_CLOSED' | 'AIR_POS_CLOSED' | 'AIR_NEG_CLOSED'

export interface TelemetryFrame {
    uptime?: number
    precharge_state?: PrechargeState
    precharge_error_flags?: PrechargeErrorFlag[]
    precharge_relay_states?: PrechargeRelay[]
    precharge_prchg_voltage?: number
    precharge_ts_voltage?: number
    precharge_online?: boolean
    [key: string]: unknown
}

export type LinkState = 'connecting' | 'open' | 'closed'

export type TelemetryStatus = 'online' | 'offline'
