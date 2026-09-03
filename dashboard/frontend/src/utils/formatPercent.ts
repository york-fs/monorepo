export function formatPercent(percent: number | undefined): string {
    return percent === undefined ? '—' : `${percent.toFixed(1)}%`
}
