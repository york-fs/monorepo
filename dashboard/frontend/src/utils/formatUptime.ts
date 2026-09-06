export function formatUptime(totalSeconds: number | undefined): string {
    if (totalSeconds === undefined || Number.isNaN(totalSeconds)) return '—'

    const seconds = Math.max(0, Math.floor(totalSeconds))
    const hours = Math.floor(seconds / 3600)
    const minutes = Math.floor((seconds % 3600) / 60)
    const secs = seconds % 60

    const pad = (n: number) => n.toString().padStart(2, '0')

    return `${pad(hours)}:${pad(minutes)}:${pad(secs)}`
}

/** Same as {@link formatUptime}, with a `.mmm` milliseconds suffix. */
export function formatUptimeWithMs(totalSeconds: number | undefined): string {
    if (totalSeconds === undefined || Number.isNaN(totalSeconds)) return '—'

    // Round to whole milliseconds *first*: rounding the fractional part on
    // its own overflows to "1000" for anything from x.9995 up, while the
    // seconds field below still shows the un-carried value.
    const totalMs = Math.round(Math.max(0, totalSeconds) * 1000)
    const ms = (totalMs % 1000).toString().padStart(3, '0')

    return `${formatUptime(totalMs / 1000)}.${ms}`
}
