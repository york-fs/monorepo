import type { InverterFaultCode } from '@/telemetry'
import type { Severity } from '@/domain/severity'
import { titleCaseEnum } from '@/utils/titleCase'

// Words naive title-casing gets wrong — see titleCaseEnum.
const ACRONYMS: Record<string, string> = {
    CAN: 'CAN',
}

// Takes `undefined` (field not arrived yet) and answers for it here rather
// than at the call site — see domain/shutdown.ts.
export function inverterFaultLabel(fault: InverterFaultCode | undefined): string {
    if (fault === undefined) return '—'
    if (fault === 'NONE') return 'None'
    return titleCaseEnum(fault, ACRONYMS)
}

/**
 * Placeholder — every non-`NONE` fault reads as critical for now.
 * `InverterFaultCode` has 10 distinct fault kinds, some of which are likely
 * less severe than others (e.g. a sensor wire fault vs. overcurrent) — a
 * real per-code severity mapping is TBD, see `plan/POWERTRAIN.md`.
 */
export function inverterFaultSeverity(fault: InverterFaultCode | undefined): Severity | undefined {
    if (fault === undefined) return undefined
    return fault === 'NONE' ? 'good' : 'critical'
}

// Placeholder wording — not yet confirmed against the inverter's own
// documentation, see `plan/POWERTRAIN.md`.
export const INVERTER_FAULT_EXPLANATIONS: Record<InverterFaultCode, string> = {
    NONE: 'No inverter faults',
    OVERVOLTAGE: 'The DC input voltage has exceeded the configured maximum',
    UNDERVOLTAGE: 'The DC input voltage has fallen below the configured minimum',
    DRIVE: 'A transistor drive error has occurred',
    OVERCURRENT: 'The AC motor current has exceeded the configured absolute maximum',
    CONTROLLER_OVERTEMPERATURE: "The controller's temperature has exceeded its configured maximum",
    MOTOR_OVERTEMPERATURE: "The motor's temperature has exceeded its configured maximum",
    SENSOR_WIRE_FAULT: 'A differential sensor wiring fault has occurred',
    SENSOR_GENERAL_FAULT: 'A sensor processing fault has occurred',
    CAN_COMMAND_FAULT: 'An invalid CAN command was received',
    ANALOG_INPUT_FAULT: 'Redundant sensor input out of range',
}

export function inverterFaultExplanation(fault: InverterFaultCode | undefined): string | undefined {
    return fault === undefined ? undefined : INVERTER_FAULT_EXPLANATIONS[fault]
}

interface OperatingRange {
    min: number
    max: number
}

// Operating ranges from you — see plan/POWERTRAIN.md.
export const INVERTER_TEMPERATURE_RANGE: OperatingRange = { min: -20, max: 85 }
export const MOTOR_TEMPERATURE_RANGE: OperatingRange = { min: -20, max: 100 }

const WARNING_MARGIN = 15 // °C before either limit
const CRITICAL_MARGIN = 5 // °C before either limit

function temperatureSeverity(celsius: number, range: OperatingRange): Severity {
    if (celsius <= range.min + CRITICAL_MARGIN || celsius >= range.max - CRITICAL_MARGIN)
        return 'critical'
    if (celsius <= range.min + WARNING_MARGIN || celsius >= range.max - WARNING_MARGIN)
        return 'warning'
    return 'good'
}

export function inverterTemperatureSeverity(celsius: number): Severity {
    return temperatureSeverity(celsius, INVERTER_TEMPERATURE_RANGE)
}

export function motorTemperatureSeverity(celsius: number): Severity {
    return temperatureSeverity(celsius, MOTOR_TEMPERATURE_RANGE)
}

// From you — see plan/POWERTRAIN.md. Unlike the temperatures, this is a
// plain in-range/out-of-range check with no warning tier.
export const INVERTER_INPUT_VOLTAGE_RANGE: OperatingRange = { min: 30, max: 800 }

export function inverterInputVoltageSeverity(volts: number): Severity {
    return volts >= INVERTER_INPUT_VOLTAGE_RANGE.min && volts <= INVERTER_INPUT_VOLTAGE_RANGE.max
        ? 'good'
        : 'critical'
}

// The motor's own hard limit (from you). Margins here are a *proportion* of
// it, unlike the temperature ranges' absolute °C offsets: "15% down from the
// redline" is the meaningful distance for a rev limit, whereas 15°C is a
// fixed physical margin whatever the range's size.
export const MOTOR_RPM_MAX = 6500

const RPM_WARNING_MARGIN = 0.15
const RPM_CRITICAL_MARGIN = 0.05

/**
 * Over-speed severity for motor RPM.
 *
 * Keyed off the motor's own 6500 RPM limit, deliberately *not* off the
 * ~5000 RPM the current pack can actually deliver: reaching the pack's
 * ceiling is normal flat-out running, not a fault, and colouring it amber
 * would break the rule the rest of the app follows — a status colour always
 * answers "is this a fault condition?" (see plan/STATUS.md). The practical
 * consequence is that this reads `good` throughout normal driving; it's a
 * silent over-speed guard rather than a performance gauge. See
 * plan/POWERTRAIN.md.
 *
 * Takes the magnitude, since `motor_rpm` is signed (reverse runs negative)
 * and over-revving backwards is still over-revving.
 */
export function rpmSeverity(rpm: number): Severity {
    const magnitude = Math.abs(rpm)
    if (magnitude >= MOTOR_RPM_MAX * (1 - RPM_CRITICAL_MARGIN)) return 'critical'
    if (magnitude >= MOTOR_RPM_MAX * (1 - RPM_WARNING_MARGIN)) return 'warning'
    return 'good'
}

const WHEEL_DIAMETER = 0.3302 // (13" diameter)
const GEAR_RATIO = 3

/** Derives car speed in MPH from motor RPM, wheel size, and gear ratio. */
export function motorRpmToMph(rpm: number): number {
    const wheelRpm = rpm / GEAR_RATIO
    const wheelSurfaceSpeed = wheelRpm * Math.PI * WHEEL_DIAMETER
    return wheelSurfaceSpeed / 26.8224
}
