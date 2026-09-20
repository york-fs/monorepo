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
