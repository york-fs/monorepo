import type {
    FuseFlag,
    InverterFaultCode,
    OnlineFlag,
    PrechargeErrorFlag,
    PrechargeRelay,
    PrechargeState,
    RtdPreventionFlag,
    ShutdownOpenCause,
    TelemetryFrame,
    TsPreventionFlag,
} from '@/telemetry'
import { FUSE_FLAGS } from '@/domain/fuses'

// Continuous values (voltages) re-sampled on every tick so charts/sparklines
// see smooth motion. Online/fuse/shutdown-cause signals only get re-rolled on
// the slower interval below — flapping those every tick would make the UI
// unreadable. The precharge state machine runs on its own clock, stepping
// through a plausible real run-through rather than random per-frame flags.
const CONTINUOUS_TICK_MS = 100
const DISCRETE_TICK_MS = 5000

const ONLINE_FLAGS: OnlineFlag[] = [
    'FRONT_ONLINE',
    'BMS_ONLINE',
    'PRECHARGE_ONLINE',
    'INVERTER_ONLINE',
]

const SHUTDOWN_CAUSES: ShutdownOpenCause[] = [
    'NONE',
    'REAR_INPUT',
    'FRONT_ESTOP',
    'BRAKE_OVER_TRAVEL',
    'INERTIA_SWITCH',
    'FRONT_AUXILIARY',
    'FRONT_OUTPUT',
    'BMS_LATCH',
    'IMD_LATCH',
    'INVERTER_INTERLOCK',
    'SHUTDOWN_LATCH_FAILURE',
    'LEFT_ESTOP',
    'RIGHT_ESTOP',
    'HVD_INTERLOCK',
    'REAR_AUXILIARY',
    'TSMS',
]

const INVERTER_FAULTS: InverterFaultCode[] = [
    'NONE',
    'OVERVOLTAGE',
    'UNDERVOLTAGE',
    'DRIVE',
    'OVERCURRENT',
    'CONTROLLER_OVERTEMPERATURE',
    'MOTOR_OVERTEMPERATURE',
    'SENSOR_WIRE_FAULT',
    'SENSOR_GENERAL_FAULT',
    'CAN_COMMAND_FAULT',
    'ANALOG_INPUT_FAULT',
]

// Relays closed in each precharge state — approximate, for demo purposes
// only (real relay states come from the backend, not derived from state).
const RELAYS_BY_STATE: Record<PrechargeState, PrechargeRelay[]> = {
    LED_CHECK: [],
    PRECHECK: ['DISCHARGE_CLOSED'],
    STANDBY: ['DISCHARGE_CLOSED'],
    PRECHARGE: ['PRECHARGE_CLOSED', 'AIR_NEG_CLOSED'],
    PRECHARGE_HOLD: ['PRECHARGE_CLOSED', 'AIR_POS_CLOSED', 'AIR_NEG_CLOSED'],
    ACTIVE: ['AIR_POS_CLOSED', 'AIR_NEG_CLOSED'],
}

// The order a real run-through steps through, looping back to LED_CHECK once
// ACTIVE ends (as if the car had been shut down and restarted).
const PRECHARGE_SEQUENCE: PrechargeState[] = [
    'LED_CHECK',
    'PRECHECK',
    'STANDBY',
    'PRECHARGE',
    'PRECHARGE_HOLD',
    'ACTIVE',
]

function randomBetween(min: number, max: number): number {
    return min + Math.random() * (max - min)
}

function randomWalk(current: number, step: number, min: number, max: number): number {
    const next = current + (Math.random() - 0.5) * step
    return Math.min(max, Math.max(min, next))
}

function pickRandom<T>(items: T[]): T {
    return items[Math.floor(Math.random() * items.length)]!
}

interface DemoState {
    onlineFlags: OnlineFlag[]
    fuseOk: FuseFlag[]
    shutdownCause: ShutdownOpenCause

    prechargeState: PrechargeState
    prechargeStateEnteredAt: number
    prechargeStateDurationMs: number
    prechargeErrorFlags: PrechargeErrorFlag[]
    // Rail voltage the PRECHARGE state spikes to, held through PRECHARGE_HOLD
    // then decayed to zero over ACTIVE (see `prechargeActiveEntryVoltage`).
    prechargeTargetVoltage: number
    prechargeActiveEntryVoltage: number
    // TS bus decays from wherever it was toward 0 once ACTIVE ends, rather
    // than snapping — anchored at the moment the loop returns to LED_CHECK.
    tsDischargeAnchorVoltage: number
    tsDischargeAnchorAt: number

    lvMin: number
    lvMax: number
    hvPackVoltage: number
    prchgVoltage: number
    tsVoltage: number
    startedAt: number

    inverterFault: InverterFaultCode
    inverterTemperature: number
    motorTemperature: number
    motorRpm: number
    pedalTravel: number
    desiredMotorCurrent: number
    motorCurrent: number
}

function enterPrechargeState(state: DemoState, next: PrechargeState, now: number) {
    if (next === 'LED_CHECK') {
        state.tsDischargeAnchorVoltage = state.tsVoltage
        state.tsDischargeAnchorAt = now
    }

    state.prechargeState = next
    state.prechargeStateEnteredAt = now

    switch (next) {
        case 'LED_CHECK':
            state.prechargeStateDurationMs = 500
            state.prechargeErrorFlags = []
            break
        case 'PRECHECK': {
            // Mostly near-instant, but occasionally waits on TS-side voltage
            // to drain first.
            const faulted = Math.random() < 0.3
            state.prechargeStateDurationMs = faulted
                ? randomBetween(2000, 4000)
                : randomBetween(100, 400)
            state.prechargeErrorFlags = faulted ? ['WAITING_DISCHARGE'] : []
            break
        }
        case 'STANDBY':
            state.prechargeStateDurationMs = randomBetween(1000, 4000)
            state.prechargeErrorFlags = ['WAITING_ACTIVATION']
            break
        case 'PRECHARGE':
            state.prechargeStateDurationMs = randomBetween(3000, 5000)
            state.prechargeErrorFlags = []
            state.prechargeTargetVoltage = state.hvPackVoltage
            break
        case 'PRECHARGE_HOLD':
            state.prechargeStateDurationMs = 500
            state.prechargeErrorFlags = []
            break
        case 'ACTIVE':
            state.prechargeStateDurationMs = randomBetween(3000, 15000)
            state.prechargeErrorFlags = []
            state.prechargeActiveEntryVoltage = state.prchgVoltage
            break
    }
}

function tickPrechargeSequence(state: DemoState, now: number) {
    if (now - state.prechargeStateEnteredAt < state.prechargeStateDurationMs) return

    const index = PRECHARGE_SEQUENCE.indexOf(state.prechargeState)
    const next = PRECHARGE_SEQUENCE[(index + 1) % PRECHARGE_SEQUENCE.length]!
    enterPrechargeState(state, next, now)
}

function tickVoltages(state: DemoState, now: number) {
    state.hvPackVoltage = randomWalk(state.hvPackVoltage, 1.5, 300, 400)
    state.lvMin = randomWalk(state.lvMin, 0.08, 10.2, 13.2)
    state.lvMax = Math.max(state.lvMin, randomWalk(state.lvMax, 0.08, state.lvMin, 13.6))

    const elapsed = now - state.prechargeStateEnteredAt

    switch (state.prechargeState) {
        case 'LED_CHECK':
        case 'PRECHECK':
        case 'STANDBY': {
            state.prchgVoltage = 0
            const sinceDischargeStart = now - state.tsDischargeAnchorAt
            state.tsVoltage = Math.max(
                0,
                state.tsDischargeAnchorVoltage * Math.exp(-sinceDischargeStart / 1000) +
                    (Math.random() - 0.5),
            )
            break
        }
        case 'PRECHARGE': {
            // Rail spikes to the pack voltage almost immediately once the
            // precharge relay closes; TS side follows an RC curve up to it.
            state.prchgVoltage = state.prechargeTargetVoltage + (Math.random() - 0.5) * 2
            const tau = 1200
            state.tsVoltage =
                state.prechargeTargetVoltage * (1 - Math.exp(-elapsed / tau)) +
                (Math.random() - 0.5) * 1.5
            break
        }
        case 'PRECHARGE_HOLD':
            state.prchgVoltage = state.prechargeTargetVoltage + (Math.random() - 0.5) * 2
            state.tsVoltage = state.prechargeTargetVoltage + (Math.random() - 0.5) * 1.5
            break
        case 'ACTIVE':
            // AIRs are closed and the precharge resistor path is bypassed, so
            // the rail this is measured on floats down to zero while the TS
            // bus itself sits at the live pack voltage.
            state.prchgVoltage = Math.max(
                0,
                state.prechargeActiveEntryVoltage * Math.exp(-elapsed / 500),
            )
            state.tsVoltage = randomWalk(
                state.tsVoltage,
                0.6,
                state.hvPackVoltage - 2,
                state.hvPackVoltage + 2,
            )
            break
    }
}

// Pedal travel wanders like a driver's input; desired motor current (APPS)
// follows it near-proportionally; actual motor current chases the desired
// figure with a lag (the inverter can't deliver instantaneously); RPM chases
// a target proportional to the current actually delivered; both temperatures
// drift up while current is high and cool down otherwise.
function tickPowertrain(state: DemoState) {
    state.pedalTravel = Math.max(0, Math.min(100, randomWalk(state.pedalTravel, 12, 0, 100)))
    state.desiredMotorCurrent = Math.max(0, (state.pedalTravel / 100) * 120 + randomBetween(-2, 2))
    state.motorCurrent = Math.max(
        0,
        state.motorCurrent +
            (state.desiredMotorCurrent - state.motorCurrent) * 0.15 +
            randomBetween(-2, 2),
    )

    const targetRpm = state.motorCurrent * 40
    state.motorRpm = state.motorRpm + (targetRpm - state.motorRpm) * 0.08

    const loadFactor = state.motorCurrent / 120
    state.inverterTemperature = Math.min(
        95,
        Math.max(
            20,
            state.inverterTemperature + loadFactor * 0.15 - 0.05 + randomBetween(-0.05, 0.05),
        ),
    )
    state.motorTemperature = Math.min(
        110,
        Math.max(
            20,
            state.motorTemperature + loadFactor * 0.12 - 0.04 + randomBetween(-0.05, 0.05),
        ),
    )
}

function isOnline(state: DemoState, flag: OnlineFlag): boolean {
    return state.onlineFlags.includes(flag)
}

function deriveTsPreventionFlags(state: DemoState): TsPreventionFlag[] {
    const flags: TsPreventionFlag[] = []
    if (!isOnline(state, 'FRONT_ONLINE')) flags.push('FRONT_OFFLINE')
    if (!isOnline(state, 'PRECHARGE_ONLINE')) flags.push('PRECHARGE_OFFLINE')
    if (state.prechargeState !== 'ACTIVE') flags.push('PRECHARGE_STATE')
    if (state.fuseOk.length < FUSE_FLAGS.length) flags.push('BAD_FUSE')
    if (state.shutdownCause !== 'NONE') flags.push('SHUTDOWN_OPEN')
    return flags
}

function deriveRtdPreventionFlags(state: DemoState): RtdPreventionFlag[] {
    const flags: RtdPreventionFlag[] = []
    if (deriveTsPreventionFlags(state).length > 0) flags.push('TS_NOT_ACTIVE')
    return flags
}

function randomizeDiscreteState(state: DemoState) {
    state.onlineFlags = ONLINE_FLAGS.filter(() => Math.random() < 0.92)
    state.fuseOk = FUSE_FLAGS.filter(() => Math.random() < 0.96)
    state.shutdownCause = Math.random() < 0.85 ? 'NONE' : pickRandom(SHUTDOWN_CAUSES.slice(1))
    state.inverterFault = Math.random() < 0.9 ? 'NONE' : pickRandom(INVERTER_FAULTS.slice(1))
}

function buildFrame(state: DemoState, now: number): TelemetryFrame {
    return {
        uptime: now - state.startedAt,
        precharge_state: state.prechargeState,
        precharge_error_flags: state.prechargeErrorFlags,
        precharge_relay_states: RELAYS_BY_STATE[state.prechargeState],
        precharge_prchg_voltage: Math.round(state.prchgVoltage),
        precharge_ts_voltage: Math.round(state.tsVoltage),
        online_flags: state.onlineFlags,
        fuses: state.fuseOk,
        lvs_min_voltage: Number(state.lvMin.toFixed(2)),
        lvs_max_voltage: Number(state.lvMax.toFixed(2)),
        shutdown_open_cause: state.shutdownCause,
        ts_prevention_flags: deriveTsPreventionFlags(state),
        rtd_prevention_flags: deriveRtdPreventionFlags(state),
        inverter_fault: state.inverterFault,
        // Same TS bus precharge reports, measured at the inverter terminals
        // instead — tracks precharge_ts_voltage with a little sensor noise.
        inverter_input_voltage: Math.round(state.tsVoltage + randomBetween(-1, 1)),
        inverter_temperature: Number(state.inverterTemperature.toFixed(1)),
        motor_temperature: Number(state.motorTemperature.toFixed(1)),
        motor_rpm: Math.round(state.motorRpm),
        motor_current: Number(state.motorCurrent.toFixed(1)),
        pedal_travel: Number(state.pedalTravel.toFixed(1)),
        desired_motor_current: Number(state.desiredMotorCurrent.toFixed(1)),
    }
}

export function isDemoMode(): boolean {
    return new URLSearchParams(window.location.search).has('demo')
}

/**
 * Starts generating synthetic telemetry frames and calls `onFrame` with each
 * one: voltages update on a fast tick, online/fuse/shutdown-cause signals
 * re-roll roughly every 5s, and the precharge state machine steps through a
 * plausible LED_CHECK → PRECHECK → STANDBY → PRECHARGE → PRECHARGE_HOLD →
 * ACTIVE run-through on its own per-state timing, looping once ACTIVE ends.
 * Returns a function that stops all timers.
 */
export function startDemoTelemetry(onFrame: (frame: TelemetryFrame) => void): () => void {
    const now = Date.now()
    const state: DemoState = {
        onlineFlags: [...ONLINE_FLAGS],
        fuseOk: [...FUSE_FLAGS],
        shutdownCause: 'NONE',
        prechargeState: 'LED_CHECK',
        prechargeStateEnteredAt: now,
        prechargeStateDurationMs: 500,
        prechargeErrorFlags: [],
        prechargeTargetVoltage: 0,
        prechargeActiveEntryVoltage: 0,
        tsDischargeAnchorVoltage: 0,
        tsDischargeAnchorAt: now,
        lvMin: 12.4,
        lvMax: 12.6,
        hvPackVoltage: 350,
        prchgVoltage: 0,
        tsVoltage: 0,
        startedAt: now,

        inverterFault: 'NONE',
        inverterTemperature: 25,
        motorTemperature: 25,
        motorRpm: 0,
        pedalTravel: 0,
        desiredMotorCurrent: 0,
        motorCurrent: 0,
    }

    onFrame(buildFrame(state, Date.now()))

    const continuousTimer = setInterval(() => {
        const tickNow = Date.now()
        tickPrechargeSequence(state, tickNow)
        tickVoltages(state, tickNow)
        tickPowertrain(state)
        onFrame(buildFrame(state, tickNow))
    }, CONTINUOUS_TICK_MS)

    const discreteTimer = setInterval(() => {
        randomizeDiscreteState(state)
    }, DISCRETE_TICK_MS)

    return () => {
        clearInterval(continuousTimer)
        clearInterval(discreteTimer)
    }
}
