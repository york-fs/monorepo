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

export type OnlineFlag = 'FRONT_ONLINE' | 'BMS_ONLINE' | 'PRECHARGE_ONLINE' | 'INVERTER_ONLINE'

export type FuseFlag =
    | 'BMS'
    | 'IMD'
    | 'TSAC_FANS'
    | 'PRECHARGE'
    | 'COOLANT_PUMP'
    | 'BRAKE_LIGHT'
    | 'TSAL_LED'
    | 'INVERTER'
    | 'SHUTDOWN_LATCH'
    | 'ENERGY_METER'
    | 'RTD_HORN'
    | 'APPS_1'
    | 'APPS_2'
    | 'FRONT'
    | 'DWIN'
    | 'AUX_1'
    | 'AUX_2'

export interface TelemetryFrame {
    uptime?: number
    precharge_state?: PrechargeState
    precharge_error_flags?: PrechargeErrorFlag[]
    precharge_relay_states?: PrechargeRelay[]
    precharge_prchg_voltage?: number
    precharge_ts_voltage?: number
    online_flags?: OnlineFlag[]
    fuses?: FuseFlag[]
    [key: string]: unknown
}

export type LinkState = 'connecting' | 'open' | 'closed'

export type TelemetryStatus = 'online' | 'offline'

/**
 * Reads a single component's online signal out of the `online_flags` array.
 * Mirrors the old per-component boolean fields (e.g. `precharge_online`):
 * `undefined` when `online_flags` itself hasn't arrived yet (no signal), a
 * plain boolean once it has.
 */
export function isFlagOnline(
    flags: readonly OnlineFlag[] | undefined,
    flag: OnlineFlag,
): boolean | undefined {
    return flags === undefined ? undefined : flags.includes(flag)
}

/**
 * Reads a single fuse's ok signal out of the `fuses` array — the array
 * lists fuses that are currently OK, so absence means blown. `undefined`
 * when `fuses` itself hasn't arrived yet (no signal), a plain boolean once
 * it has.
 */
export function isFuseOk(
    flags: readonly FuseFlag[] | undefined,
    flag: FuseFlag,
): boolean | undefined {
    return flags === undefined ? undefined : flags.includes(flag)
}
