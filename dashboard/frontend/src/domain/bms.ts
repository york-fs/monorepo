import type { BmsMasterErrorFlag } from '@/telemetry'
import type { Severity } from '@/domain/severity'

interface BmsErrorMeta {
    label: string
    description: string
}

/**
 * Label + explanation per master error flag, same treatment as
 * `PRECHARGE_FLAG_META` and `INVERTER_FAULT_EXPLANATIONS`. Key order is the
 * render order — roughly "can the master run at all" first, then measurement
 * faults, then segment-level problems — rather than the backend's enum order,
 * which the `Record` type still forces every flag to appear in exactly once.
 *
 * Wording is a placeholder, not yet checked against the BMS firmware — see
 * plan/BMS.md.
 */
export const BMS_ERROR_META: Record<BmsMasterErrorFlag, BmsErrorMeta> = {
    CAN_OFFLINE: {
        label: 'CAN Offline',
        description: 'The BMS has lost its CAN bus connection',
    },
    NO_CONFIG: {
        label: 'No Config',
        description: 'The BMS has no usable config',
    },
    DEADLINE_OVERRUN: {
        label: 'Deadline Overrun',
        description: 'A monitoring task has overrun its timing deadline',
    },
    BAD_REFERENCE: {
        label: 'Bad Reference',
        description: 'The onboard voltage reference read outside its expected range',
    },
    BAD_CURRENT_SENSOR: {
        label: 'Bad Current Sensor',
        description: "A current sensor's zero voltage is bad",
    },
    OVERTEMPERATURE: {
        label: 'Overtemperature',
        description: 'The master board is overtemperature',
    },
    OVERCURRENT_THRESHOLD: {
        label: 'Overcurrent Threshold',
        description:
            "An overcurrent has been detected by one or both of the current sensors' overcurrent pins",
    },
    OVERCURRENT_MEASURED: {
        label: 'Overcurrent Measured',
        description: 'An overcurrent has been measured by one or both current sensors',
    },
    SEGMENT_ERROR: {
        label: 'Segment Error',
        description: 'One or more segments have one or more errors',
    },
    BAD_SEGMENT_COUNT: {
        label: 'Bad Segment Count',
        description: 'The number of responding segments does not match the configured count',
    },
}

export const BMS_ERROR_ORDER = Object.keys(BMS_ERROR_META) as BmsMasterErrorFlag[]

/**
 * Whole-field severity: any raised error is critical, none is good.
 * Placeholder in the same sense as `inverterFaultSeverity` — some of these
 * (a deadline overrun, a CAN dropout) are plausibly a warning rather than a
 * fault, but that split needs the firmware's own view. See plan/BMS.md.
 */
export function bmsErrorSeverity(
    flags: readonly BmsMasterErrorFlag[] | undefined,
): Severity | undefined {
    if (flags === undefined) return undefined
    return flags.length === 0 ? 'good' : 'critical'
}

interface OperatingRange {
    min: number
    max: number
}

// From you — see plan/BMS.md.
export const CELL_VOLTAGE_RANGE: OperatingRange = { min: 3, max: 4.2 }

// Absolute margins, like the powertrain temperatures rather than the
// proportional rev-limit margins: a cell's headroom is a fixed number of
// millivolts from its limits, not a percentage of the window.
const CELL_VOLTAGE_WARNING_MARGIN = 0.15 // V from either limit
const CELL_VOLTAGE_CRITICAL_MARGIN = 0.05 // V from either limit

export function cellVoltageSeverity(volts: number): Severity {
    if (
        volts <= CELL_VOLTAGE_RANGE.min + CELL_VOLTAGE_CRITICAL_MARGIN ||
        volts >= CELL_VOLTAGE_RANGE.max - CELL_VOLTAGE_CRITICAL_MARGIN
    )
        return 'critical'
    if (
        volts <= CELL_VOLTAGE_RANGE.min + CELL_VOLTAGE_WARNING_MARGIN ||
        volts >= CELL_VOLTAGE_RANGE.max - CELL_VOLTAGE_WARNING_MARGIN
    )
        return 'warning'
    return 'good'
}

/**
 * One-sided, unlike the powertrain temperatures: you gave an upper limit
 * only, and a cold pack isn't a fault the way a cold inverter notionally is.
 * Same 15°C/5°C margins as `domain/powertrain.ts` so the two temperature
 * readouts change colour at comparable distances from their limits.
 */
export const CELL_TEMPERATURE_MAX = 60

const CELL_TEMPERATURE_WARNING_MARGIN = 15 // °C below the limit
const CELL_TEMPERATURE_CRITICAL_MARGIN = 5 // °C below the limit

export function cellTemperatureSeverity(celsius: number): Severity {
    if (celsius >= CELL_TEMPERATURE_MAX - CELL_TEMPERATURE_CRITICAL_MARGIN) return 'critical'
    if (celsius >= CELL_TEMPERATURE_MAX - CELL_TEMPERATURE_WARNING_MARGIN) return 'warning'
    return 'good'
}

/**
 * Pack current limit, applied to the magnitude — both sensors read positive
 * while driving and negative while charging or regenerating, and 200A the
 * wrong way is as far out of spec as 200A the right way.
 *
 * Proportional margins, like `rpmSeverity`'s: "within 5% of the limit" is the
 * meaningful distance from a current ceiling, the same way it is from a rev
 * limit.
 */
export const PACK_CURRENT_MAX = 200

const CURRENT_WARNING_MARGIN = 0.15 // fraction of the limit
const CURRENT_CRITICAL_MARGIN = 0.05 // fraction of the limit

export function packCurrentSeverity(amps: number): Severity {
    const magnitude = Math.abs(amps)
    if (magnitude >= PACK_CURRENT_MAX * (1 - CURRENT_CRITICAL_MARGIN)) return 'critical'
    if (magnitude >= PACK_CURRENT_MAX * (1 - CURRENT_WARNING_MARGIN)) return 'warning'
    return 'good'
}

/**
 * A debugging aid rather than a fault condition: any i2c error at all means
 * the bus to the segments is unhappy and is worth knowing about, but it isn't
 * critical on its own — a real communication failure surfaces as
 * `SEGMENT_ERROR`/`BAD_SEGMENT_COUNT` above.
 */
export function i2cErrorCountSeverity(count: number): Severity {
    return count === 0 ? 'good' : 'warning'
}
