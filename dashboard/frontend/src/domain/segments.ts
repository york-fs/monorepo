import type { SegmentErrorFlag, SegmentReadings } from '@/telemetry'
import { severityOf, worstSeverity, type Severity } from '@/domain/severity'
import { cellTemperatureSeverity, cellVoltageSeverity } from '@/domain/bms'

/**
 * The pack's fitted segments, by the letter each is labelled with on the pack
 * drawing (assets/bms/pack-top.svg). Segment id 0 is A. The BMS has ten slots;
 * the rest always read disconnected and aren't shown.
 */
export const SEGMENT_LETTERS = ['A', 'B', 'C', 'D', 'E', 'F'] as const

/** Cells fitted per segment, as drawn in assets/bms/segment-sides.svg. */
export const FITTED_CELL_COUNT = 14

/**
 * The first thermistors a segment reports are on its board, around the passive
 * balancing circuitry, rather than on the cells. Matches
 * `k_balance_thermistor_count` in bms/common.hh.
 */
export const BALANCER_THERMISTOR_COUNT = 4

export const SEGMENT_ERROR_META: Record<SegmentErrorFlag, { label: string; description: string }> =
    {
        DISCONNECTED: {
            label: 'Disconnected',
            description: 'The segment is disconnected and not responding',
        },
        BAD_CELL_COUNT: {
            label: 'Bad Cell Count',
            description: "The number of connected cells doesn't match the master's config",
        },
        BAD_THERMISTOR_COUNT: {
            label: 'Bad Thermistor Count',
            description: "Fewer thermistors are connected than the master's config expects",
        },
        BAD_RAIL_VOLTAGE: {
            label: 'Bad Rail Voltage',
            description: "The segment's power rail voltage is out of tolerance",
        },
        UNRELIABLE_COMMUNICATION: {
            label: 'Unreliable Communication',
            description: 'The segment sent the wrong number of bytes or corrupted data',
        },
        UNRELIABLE_MEASUREMENT: {
            label: 'Unreliable Measurement',
            description:
                'Readings are unreliable from degraded cell voltages or a bad rail voltage',
        },
        UNDERVOLTAGE: { label: 'Undervoltage', description: 'One or more cells is undervoltage' },
        OVERVOLTAGE: { label: 'Overvoltage', description: 'One or more cells is overvoltage' },
        UNDERTEMPERATURE: {
            label: 'Undertemperature',
            description: 'One or more cells is undertemperature',
        },
        OVERTEMPERATURE: {
            label: 'Overtemperature',
            description: 'One or more cells is overtemperature',
        },
        BALANCER_OVERTEMPERATURE: {
            label: 'Balancer Overtemperature',
            description: "The segment's passive balancing circuitry is overtemperature",
        },
    }

/** Any raised segment error is critical, the same as the master's errors. */
export function segmentErrorSeverity(
    flags: readonly SegmentErrorFlag[] | undefined,
): Severity | undefined {
    if (flags === undefined) return undefined
    return flags.length === 0 ? 'good' : 'critical'
}

export interface SegmentStats {
    minCellVoltage?: number
    maxCellVoltage?: number
    maxTemperature?: number
    maxBalancerTemperature?: number
}

function present(values: readonly (number | null)[]): number[] {
    return values.filter((value): value is number => value !== null)
}

/** Extremes across a segment's fitted cells and connected cell thermistors. */
export function segmentStats(readings: SegmentReadings | undefined): SegmentStats {
    const cells = present(readings?.cell_voltages.slice(0, FITTED_CELL_COUNT) ?? [])
    const temperatures = present(readings?.temperatures.slice(BALANCER_THERMISTOR_COUNT) ?? [])
    const balancer = present(readings?.temperatures.slice(0, BALANCER_THERMISTOR_COUNT) ?? [])
    return {
        minCellVoltage: cells.length > 0 ? Math.min(...cells) : undefined,
        maxCellVoltage: cells.length > 0 ? Math.max(...cells) : undefined,
        maxTemperature: temperatures.length > 0 ? Math.max(...temperatures) : undefined,
        maxBalancerTemperature: balancer.length > 0 ? Math.max(...balancer) : undefined,
    }
}

export interface PackCellStats {
    lowest?: { segment: number; cell: number; volts: number }
    highest?: number
}

/** The lowest fitted cell, and the highest voltage, across the fitted segments. */
export function packCellStats(readings: ReadonlyMap<number, SegmentReadings>): PackCellStats {
    const stats: PackCellStats = {}
    for (const [segment, { cell_voltages }] of readings) {
        if (segment >= SEGMENT_LETTERS.length) continue
        cell_voltages.slice(0, FITTED_CELL_COUNT).forEach((volts, cell) => {
            if (volts === null) return
            if (stats.lowest === undefined || volts < stats.lowest.volts) {
                stats.lowest = { segment, cell, volts }
            }
            stats.highest = Math.max(stats.highest ?? volts, volts)
        })
    }
    return stats
}

/** A segment's overall state: its own errors and its worst reading. */
export function segmentSeverity(
    flags: readonly SegmentErrorFlag[] | undefined,
    readings: SegmentReadings | undefined,
): Severity | undefined {
    const { minCellVoltage, maxCellVoltage, maxTemperature } = segmentStats(readings)
    return worstSeverity(
        segmentErrorSeverity(flags),
        severityOf(minCellVoltage, cellVoltageSeverity),
        severityOf(maxCellVoltage, cellVoltageSeverity),
        severityOf(maxTemperature, cellTemperatureSeverity),
    )
}

/**
 * Which BMS thermistor index (4–23, in the order the segment reports them,
 * after the balancer ones) is each thermistor on the segment drawing.
 *
 * PROVISIONAL: the firmware only knows MUX positions (see `k_thermistor_mapping`
 * in bms/segment.cc), not the physical labels, so this is a placeholder
 * guess. Confirm it on the car, for example by warming one thermistor and
 * watching which index moves, then set `THERMISTOR_MAPPING_CONFIRMED`.
 */
export const THERMISTOR_INDEX: Record<string, number> = {
    'A-T1': 4,
    'A-T2': 5,
    'A-T3': 6,
    'A-T4': 7,
    'A-T5': 8,
    'A-T6': 9,
    'A-T7': 10,
    'B-T1': 11,
    'B-T2': 12,
    'B-T3': 13,
    'B-T4': 14,
    'B-T5': 15,
    'B-T6': 16,
    'B-T7': 17,
}
export const THERMISTOR_MAPPING_CONFIRMED = false

// Segment errors are logged per segment, as SEGMENT_<letter>_<flag> (see
// dashboard/events.py).
const SEGMENT_ERROR_CODE = /^SEGMENT_([A-Z])_(\w+)$/

/** What a logged segment error code means, or undefined if it isn't one. */
export function segmentErrorCodeDescription(code: string): string | undefined {
    const flag = SEGMENT_ERROR_CODE.exec(code)?.[2] as SegmentErrorFlag | undefined
    return flag && SEGMENT_ERROR_META[flag]?.description
}
