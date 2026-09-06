/**
 * Cleans up a `SCREAMING_SNAKE_CASE` enum member for display —
 * `FRONT_ESTOP` → `Front E-Stop`, `CAN_COMMAND_FAULT` → `CAN Command Fault`.
 *
 * `acronyms` maps a raw word to its preferred rendering, for the words naive
 * title-casing gets wrong: initialisms that should stay upper-case (`BMS`,
 * `CAN`) and words with punctuation (`ESTOP` → `E-Stop`). Each domain passes
 * its own dictionary, since the same enum vocabulary isn't shared between
 * them (see domain/shutdown.ts and domain/powertrain.ts).
 */
export function titleCaseEnum(name: string, acronyms: Record<string, string> = {}): string {
    return name
        .split('_')
        .map((word) => acronyms[word] ?? word.charAt(0) + word.slice(1).toLowerCase())
        .join(' ')
}
