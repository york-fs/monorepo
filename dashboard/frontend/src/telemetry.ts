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
    | 'RATE_LIMIT'

export type PrechargeRelay =
    'DISCHARGE_CLOSED' | 'PRECHARGE_CLOSED' | 'AIR_POS_CLOSED' | 'AIR_NEG_CLOSED'

export type OnlineFlag = 'FRONT_ONLINE' | 'BMS_ONLINE' | 'PRECHARGE_ONLINE' | 'INVERTER_ONLINE'

export type ShutdownOpenCause =
    | 'NONE'
    | 'REAR_INPUT'
    | 'FRONT_ESTOP'
    | 'BRAKE_OVER_TRAVEL'
    | 'INERTIA_SWITCH'
    | 'FRONT_AUXILIARY'
    | 'FRONT_OUTPUT'
    | 'BMS_LATCH'
    | 'IMD_LATCH'
    | 'INVERTER_INTERLOCK'
    | 'SHUTDOWN_LATCH_FAILURE'
    | 'LEFT_ESTOP'
    | 'RIGHT_ESTOP'
    | 'HVD_INTERLOCK'
    | 'REAR_AUXILIARY'
    | 'TSMS'

export type TsPreventionFlag =
    | 'SHUTDOWN_OPEN'
    | 'BAD_FUSE'
    | 'FRONT_OFFLINE'
    | 'NOT_REQUESTED'
    | 'PRECHARGE_OFFLINE'
    | 'PRECHARGE_STATE'

export type RtdPreventionFlag = 'TS_NOT_ACTIVE' | 'NOT_REQUESTED' | 'BRAKE_NOT_PRESSED'

export type InverterFaultCode =
    | 'NONE'
    | 'OVERVOLTAGE'
    | 'UNDERVOLTAGE'
    | 'DRIVE'
    | 'OVERCURRENT'
    | 'CONTROLLER_OVERTEMPERATURE'
    | 'MOTOR_OVERTEMPERATURE'
    | 'SENSOR_WIRE_FAULT'
    | 'SENSOR_GENERAL_FAULT'
    | 'CAN_COMMAND_FAULT'
    | 'ANALOG_INPUT_FAULT'

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
    lvs_min_voltage?: number
    lvs_max_voltage?: number
    shutdown_open_cause?: ShutdownOpenCause
    ts_prevention_flags?: TsPreventionFlag[]
    rtd_prevention_flags?: RtdPreventionFlag[]
    inverter_fault?: InverterFaultCode
    inverter_temperature?: number
    motor_temperature?: number
    inverter_input_voltage?: number
    motor_current?: number
    motor_rpm?: number
    pedal_travel?: number
    desired_motor_current?: number
    [key: string]: unknown
}

export type LinkState = 'connecting' | 'open' | 'closed'

export type TelemetryStatus = 'online' | 'offline'

/**
 * Reads a single flag's presence out of a `Flag`-enum wire array (all of
 * these serialize as an array of set member names). `undefined` when the
 * array itself hasn't arrived yet (no signal), a plain boolean once it has —
 * every per-flag helper below is a thin, differently-named wrapper over this,
 * since what "presence" means (online vs. ok vs. blocking) differs per
 * field.
 */
function isFlagSet<T extends string>(
    flags: readonly T[] | undefined,
    flag: T,
): boolean | undefined {
    return flags === undefined ? undefined : flags.includes(flag)
}

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
    return isFlagSet(flags, flag)
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
    return isFlagSet(flags, flag)
}

/**
 * Reads whether a single condition is currently blocking TS activation out
 * of the `ts_prevention_flags` array — presence means "this is blocking",
 * absence means the condition is satisfied. `undefined` when the array
 * hasn't arrived yet (no signal).
 */
export function isTsPreventionFlagSet(
    flags: readonly TsPreventionFlag[] | undefined,
    flag: TsPreventionFlag,
): boolean | undefined {
    return isFlagSet(flags, flag)
}

/**
 * Same as `isTsPreventionFlagSet`, for `rtd_prevention_flags`.
 */
export function isRtdPreventionFlagSet(
    flags: readonly RtdPreventionFlag[] | undefined,
    flag: RtdPreventionFlag,
): boolean | undefined {
    return isFlagSet(flags, flag)
}
