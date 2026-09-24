export function formatAmps(amps: number | undefined, decimals = 1): string {
    if (amps === undefined) return '—'

    // Round first, then re-format — `toFixed` keeps the sign of a small
    // negative ("-0.04" -> "-0.0"), which reads as a bogus reading rather
    // than as zero. Same reason as formatVolts.
    const rounded = Number(amps.toFixed(decimals))
    return `${(rounded === 0 ? 0 : rounded).toFixed(decimals)} A`
}
