import type { InverterFaultCode } from '@/telemetry'

// Acronyms/initialisms that shouldn't get naive title-casing.
const ACRONYMS: Record<string, string> = {
    CAN: 'CAN',
}

function titleCaseWord(word: string) {
    return ACRONYMS[word] ?? word.charAt(0) + word.slice(1).toLowerCase()
}

export function inverterFaultLabel(fault: InverterFaultCode): string {
    if (fault === 'NONE') return 'None'
    return fault.split('_').map(titleCaseWord).join(' ')
}

/**
 * Placeholder — every non-`NONE` fault reads as critical for now.
 * `InverterFaultCode` has 10 distinct fault kinds, some of which are likely
 * less severe than others (e.g. a sensor wire fault vs. overcurrent) — a
 * real per-code severity mapping is TBD, see `plan/POWERTRAIN.md`.
 */
export function inverterFaultSeverity(fault: InverterFaultCode): 'good' | 'critical' {
    return fault === 'NONE' ? 'good' : 'critical'
}

// Placeholder wording — not yet confirmed against the inverter's own
// documentation, see `plan/POWERTRAIN.md`.
export const INVERTER_FAULT_EXPLANATIONS: Record<InverterFaultCode, string> = {
    NONE: 'No inverter faults',
    OVERVOLTAGE: 'The DC input voltage has exceeded the configured maximum',
    UNDERVOLTAGE: 'The DC input voltage has fallen below the configured minimum',
    DRIVE: 'A transistor drive error has occured',
    OVERCURRENT: 'The AC motor current has exceeded the configured absolute maximum',
    CONTROLLER_OVERTEMPERATURE: "The controller's temperature has exceeded its configured maximum",
    MOTOR_OVERTEMPERATURE: "The motor's temperature has exceeded its configured maximum",
    SENSOR_WIRE_FAULT: 'A differential sensor wiring fault has occured',
    SENSOR_GENERAL_FAULT: 'A sensor processing fault has occured',
    CAN_COMMAND_FAULT: 'An invalid CAN command was received',
    ANALOG_INPUT_FAULT: 'Redundant sensor input out of range',
}

export function inverterFaultExplanation(fault: InverterFaultCode): string {
    return INVERTER_FAULT_EXPLANATIONS[fault]
}

export type TemperatureSeverity = 'good' | 'warning' | 'critical'

interface OperatingRange {
    min: number
    max: number
}

// Operating ranges from you — see plan/POWERTRAIN.md.
export const INVERTER_TEMPERATURE_RANGE: OperatingRange = { min: -20, max: 85 }
export const MOTOR_TEMPERATURE_RANGE: OperatingRange = { min: -20, max: 100 }

const WARNING_MARGIN = 15 // °C before either limit
const CRITICAL_MARGIN = 5 // °C before either limit

function temperatureSeverity(celsius: number, range: OperatingRange): TemperatureSeverity {
    if (celsius <= range.min + CRITICAL_MARGIN || celsius >= range.max - CRITICAL_MARGIN)
        return 'critical'
    if (celsius <= range.min + WARNING_MARGIN || celsius >= range.max - WARNING_MARGIN)
        return 'warning'
    return 'good'
}

export function inverterTemperatureSeverity(celsius: number): TemperatureSeverity {
    return temperatureSeverity(celsius, INVERTER_TEMPERATURE_RANGE)
}

export function motorTemperatureSeverity(celsius: number): TemperatureSeverity {
    return temperatureSeverity(celsius, MOTOR_TEMPERATURE_RANGE)
}

// From you — see plan/POWERTRAIN.md. Unlike the temperatures, this is a
// plain in-range/out-of-range check with no warning tier.
export const INVERTER_INPUT_VOLTAGE_RANGE: OperatingRange = { min: 30, max: 800 }

export function inverterInputVoltageSeverity(volts: number): 'good' | 'critical' {
    return volts >= INVERTER_INPUT_VOLTAGE_RANGE.min && volts <= INVERTER_INPUT_VOLTAGE_RANGE.max
        ? 'good'
        : 'critical'
}

const WHEEL_DIAMETER = 0.3302 // (13" diameter)
const GEAR_RATIO = 3

/** Derives car speed in MPH from motor RPM, wheel size, and gear ratio. */
export function motorRpmToMph(rpm: number): number {
    const wheelRpm = rpm / GEAR_RATIO
    const wheelSurfaceSpeed = wheelRpm * Math.PI * WHEEL_DIAMETER
    return wheelSurfaceSpeed / 26.8224
}
