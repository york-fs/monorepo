/**
 * The three-colour state scale every accent in the app keys off — tiles
 * (`Tile`), the min/max arrow pair (`MinMaxSub`), the fuse grid and the
 * activation checklists.
 *
 * Callers own the mapping from a reading to one of these (see
 * `lvs.ts`, `powertrain.ts`); the shared components only ever know the
 * scale itself, never what a value means. `undefined` wherever a severity is
 * accepted means "no signal yet" and stays uncoloured.
 */
export type Severity = 'good' | 'warning' | 'critical'

const RANK: Record<Severity, number> = { good: 0, warning: 1, critical: 2 }

/** The worst of several severities, ignoring any still without a signal. */
export function worstSeverity(...severities: (Severity | undefined)[]): Severity | undefined {
    return severities.reduce<Severity | undefined>(
        (worse, next) =>
            next === undefined || (worse !== undefined && RANK[worse] >= RANK[next]) ? worse : next,
        undefined,
    )
}

/** The severity of a reading that may not have arrived yet. */
export function severityOf(
    value: number | null | undefined,
    rate: (value: number) => Severity,
): Severity | undefined {
    return value === null || value === undefined ? undefined : rate(value)
}
