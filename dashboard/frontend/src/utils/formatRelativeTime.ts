export function formatRelativeTime(totalSeconds: number | undefined): string {
    if (totalSeconds === undefined || Number.isNaN(totalSeconds)) return '—'

    const seconds = Math.max(0, Math.floor(totalSeconds))
    const pluralize = (value: number, unit: string) =>
        `${value} ${unit}${value === 1 ? '' : 's'} ago`

    if (seconds < 60) return pluralize(seconds, 'second')

    const minutes = Math.floor(seconds / 60)
    if (minutes < 60) return pluralize(minutes, 'minute')

    const hours = Math.floor(minutes / 60)
    return pluralize(hours, 'hour')
}
