import type { ShutdownOpenCause } from '@/types/telemetry'

// Acronyms/initialisms that shouldn't get naive title-casing.
const ACRONYMS: Record<string, string> = {
    BMS: 'BMS',
    IMD: 'IMD',
    TSMS: 'TSMS',
    HVD: 'HVD',
    ESTOP: 'E-Stop',
}

function titleCaseWord(word: string) {
    return ACRONYMS[word] ?? word.charAt(0) + word.slice(1).toLowerCase()
}

export function shutdownOpenCauseLabel(cause: ShutdownOpenCause): string {
    if (cause === 'NONE') return 'None'
    return cause.split('_').map(titleCaseWord).join(' ')
}

export function shutdownOpenCauseSeverity(cause: ShutdownOpenCause): 'good' | 'critical' {
    return cause === 'NONE' ? 'good' : 'critical'
}
