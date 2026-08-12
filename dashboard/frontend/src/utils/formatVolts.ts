export function formatVolts(volts: number | undefined): string {
    return volts === undefined ? '—' : `${Math.round(volts)} V`
}
