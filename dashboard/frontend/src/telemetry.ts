/**
 * The wire vocabularies, each declared as a runtime array with its type
 * derived from it rather than as a hand-written union alongside a matching
 * array.
 *
 * The arrays have to exist regardless: a TS union is erased at runtime, so
 * enumerating a flag enum's members — to render a cell per fuse, a step per
 * precharge state, a checklist row per prevention flag — needs a real value.
 * Deriving the type from the array means each member is written once here
 * instead of once in a union and again in a `domain/` array that had to be
 * kept in step by hand.
 *
 * Declaration order mirrors the backend's own enum order. Where the UI wants
 * a different order that's the consumer's business, not this file's — see
 * `domain/prevention.ts`.
 */

export const PRECHARGE_STATES = [
    'LED_CHECK',
    'PRECHECK',
    'STANDBY',
    'PRECHARGE',
    'PRECHARGE_HOLD',
    'ACTIVE',
] as const
export type PrechargeState = (typeof PRECHARGE_STATES)[number]

export const PRECHARGE_ERROR_FLAGS = [
    'DISCHARGE_OPEN',
    'PRECHARGE_CLOSED',
    'AIR_POS_CLOSED',
    'AIR_NEG_CLOSED',
    'PRECHECK_VOLTAGE',
    'WAITING_DISCHARGE',
    'WAITING_ACTIVATION',
    'SHUTDOWN_OPEN',
    'PRECHARGE_OPEN',
    'AIR_POS_OPEN',
    'AIR_NEG_OPEN',
    'DEACTIVATION',
    'DEVIATION',
    'RATE_LIMIT',
] as const
export type PrechargeErrorFlag = (typeof PRECHARGE_ERROR_FLAGS)[number]

export const PRECHARGE_RELAYS = [
    'DISCHARGE_CLOSED',
    'PRECHARGE_CLOSED',
    'AIR_POS_CLOSED',
    'AIR_NEG_CLOSED',
] as const
export type PrechargeRelay = (typeof PRECHARGE_RELAYS)[number]

export const ONLINE_FLAGS = [
    'FRONT_ONLINE',
    'BMS_ONLINE',
    'PRECHARGE_ONLINE',
    'INVERTER_ONLINE',
] as const
export type OnlineFlag = (typeof ONLINE_FLAGS)[number]

export const SHUTDOWN_OPEN_CAUSES = [
    'NONE',
    'REAR_INPUT',
    'FRONT_ESTOP',
    'BRAKE_OVER_TRAVEL',
    'INERTIA_SWITCH',
    'FRONT_AUXILIARY',
    'FRONT_OUTPUT',
    'BMS_LATCH',
    'IMD_LATCH',
    'INVERTER_INTERLOCK',
    'SHUTDOWN_LATCH_FAILURE',
    'LEFT_ESTOP',
    'RIGHT_ESTOP',
    'HVD_INTERLOCK',
    'REAR_AUXILIARY',
    'TSMS',
] as const
export type ShutdownOpenCause = (typeof SHUTDOWN_OPEN_CAUSES)[number]

export const TS_PREVENTION_FLAGS = [
    'SHUTDOWN_OPEN',
    'BAD_FUSE',
    'FRONT_OFFLINE',
    'NOT_REQUESTED',
    'PRECHARGE_OFFLINE',
    'PRECHARGE_STATE',
    'INVERTER_OFFLINE',
    'INVERTER_FAULT',
    'BMS_OFFLINE',
    'BMS_FAULT',
] as const
export type TsPreventionFlag = (typeof TS_PREVENTION_FLAGS)[number]

export const RTD_PREVENTION_FLAGS = [
    'TS_NOT_ACTIVE',
    'NOT_REQUESTED',
    'APPS_NOT_CALIBRATED',
    'BRAKE_NOT_PRESSED',
] as const
export type RtdPreventionFlag = (typeof RTD_PREVENTION_FLAGS)[number]

export const INVERTER_FAULT_CODES = [
    'NONE',
    'OVERVOLTAGE',
    'UNDERVOLTAGE',
    'DRIVE',
    'OVERCURRENT',
    'CONTROLLER_OVERTEMPERATURE',
    'MOTOR_OVERTEMPERATURE',
    'SENSOR_WIRE_FAULT',
    'SENSOR_GENERAL_FAULT',
    'CAN_COMMAND_FAULT',
    'ANALOG_INPUT_FAULT',
] as const
export type InverterFaultCode = (typeof INVERTER_FAULT_CODES)[number]

export const BMS_MASTER_ERROR_FLAGS = [
    'CAN_OFFLINE',
    'NO_CONFIG',
    'DEADLINE_OVERRUN',
    'BAD_REFERENCE',
    'OVERTEMPERATURE',
    'BAD_CURRENT_SENSOR',
    'OVERCURRENT_THRESHOLD',
    'OVERCURRENT_MEASURED',
    'SEGMENT_ERROR',
    'BAD_SEGMENT_COUNT',
] as const
export type BmsMasterErrorFlag = (typeof BMS_MASTER_ERROR_FLAGS)[number]

export const FUSE_FLAGS = [
    'BMS',
    'IMD',
    'TSAC_FANS',
    'PRECHARGE',
    'COOLANT_PUMP',
    'BRAKE_LIGHT',
    'TSAL_LED',
    'INVERTER',
    'SHUTDOWN_LATCH',
    'ENERGY_METER',
    'RTD_HORN',
    'APPS_1',
    'APPS_2',
    'FRONT',
    'DWIN',
    'AUX_1',
    'AUX_2',
] as const
export type FuseFlag = (typeof FUSE_FLAGS)[number]

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
    bms_master_error_flags?: BmsMasterErrorFlag[]
    bms_i2c_error_count?: number
    positive_current?: number
    negative_current?: number
    min_cell_voltage?: number
    max_cell_voltage?: number
    min_cell_temperature?: number
    max_cell_temperature?: number
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

/**
 * Reads whether a single BMS master error is currently raised out of the
 * `bms_master_error_flags` array — presence means the error is active.
 * `undefined` when the array hasn't arrived yet (no signal).
 */
export function isBmsErrorFlagSet(
    flags: readonly BmsMasterErrorFlag[] | undefined,
    flag: BmsMasterErrorFlag,
): boolean | undefined {
    return isFlagSet(flags, flag)
}
