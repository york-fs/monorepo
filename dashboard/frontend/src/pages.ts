/**
 * The dashboard's pages, in tab order. Declared once here so the router and
 * the tab bar can't drift apart.
 */
export const PAGES = [
    { name: 'overview', path: '/', label: 'Overview' },
    { name: 'predrive', path: '/predrive', label: 'Pre-drive' },
    { name: 'drive', path: '/drive', label: 'Drive' },
    { name: 'bms', path: '/bms', label: 'BMS' },
    { name: 'debug', path: '/debug', label: 'Debug' },
] as const
export type PageName = (typeof PAGES)[number]['name']
