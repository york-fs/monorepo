export function formatCelsius(celsius: number | undefined): string {
    return celsius === undefined ? '—' : `${Math.round(celsius)}°C`
}
