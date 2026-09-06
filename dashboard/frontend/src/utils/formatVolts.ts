export function formatVolts(volts: number | undefined, decimals = 0): string {
    if (volts === undefined) return '—'

    // Round first, then re-format: `toFixed` keeps the sign of a small
    // negative ("-0.4" -> "-0"), which reads as a bogus reading rather than
    // as zero.
    const rounded = Number(volts.toFixed(decimals))
    return `${(rounded === 0 ? 0 : rounded).toFixed(decimals)} V`
}
