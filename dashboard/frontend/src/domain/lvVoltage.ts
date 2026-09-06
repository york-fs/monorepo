import type { Severity } from '@/domain/severity'

/** Inclusive volt ranges — see plan/DISTRIBUTION.md for the source. */
export const LV_VOLTAGE_GOOD_RANGE: [number, number] = [11.5, 13]
export const LV_VOLTAGE_WARNING_RANGE: [number, number] = [10.5, 14]

export function lvVoltageSeverity(volts: number): Severity {
    if (volts >= LV_VOLTAGE_GOOD_RANGE[0] && volts <= LV_VOLTAGE_GOOD_RANGE[1]) return 'good'
    if (volts >= LV_VOLTAGE_WARNING_RANGE[0] && volts <= LV_VOLTAGE_WARNING_RANGE[1])
        return 'warning'
    return 'critical'
}
