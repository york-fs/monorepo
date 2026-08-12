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

    const clamped = Math.max(0, totalSeconds)
    const ms = Math.round((clamped % 1) * 1000)
        .toString()
        .padStart(3, '0')

    return `${formatUptime(clamped)}.${ms}`
}
