#import "@preview/unify:0.8.1": unit
#import "../abbrevs.typ": abbr
#import "../util.typ"

= DTI HV-550 Inverter

== CAN Messages

#let gd1-fields = (
    (
        bytes: (0, 3),
        name: "ERPM",
        type: "int32_t",
        desc: [The measured electrical RPM of the motor.]
    ),
    (
        bytes: (4, 5),
        name: "DUTY_CYCLE",
        type: "int16_t",
        desc: [The currently set duty cycle in tenths of a #unit("percent"). A negative value represents regeneration.]
    ),
    (
        bytes: (6, 7),
        name: "INPUT_VOLTAGE",
        type: "int16_t",
        desc: [The measured DC input voltage in #unit("V").]
    ),
)

#let gd2-fields = (
    (
        bytes: (0, 1),
        name: "AC_CURRENT",
        type: "int16_t",
        desc: [The measured motor current in tenths of an #unit("A"). A negative value represents regeneration.],
    ),
    (
        bytes: (2, 3),
        name: "DC_CURRENT",
        type: "int16_t",
        desc: [The measured battery current in tenths of an #unit("A"). A negative value represents regeneration.],
    ),
)

// TODO: Make a util function for this.
#let fault-status(name, content) = [
    #raw(name)
    #linebreak()
    #box(inset: (left: 1.5em))[#content]
]

#let gd3-fields = (
    (
        bytes: (0, 1),
        name: "CTRL_TEMP",
        type: "int16_t",
        desc: [The measured temperature of the inverter's semiconductor in tenths of a #unit("dC").],
    ),
    (
        bytes: (2, 3),
        name: "MOTOR_TEMP",
        type: "int16_t",
        desc: [The measured motor temperature in tenths of a #unit("dC").],
    ),
    (
        byte: 4,
        name: "FAULT",
        type: "enum",
        desc: enum(
            numbering: i => [#(i - 1):],
            raw("NO_FAULTS"),
            fault-status("OVERVOLTAGE")[
                DC input voltage higher than the configured maximum.
            ],
            fault-status("UNDERVOLTAGE")[
                DC input voltage lower than the configured minimum.
            ],
            fault-status("DRV")[
                Transistor drive error.
            ],
            fault-status("OVERCURRENT")[
                AC motor current higher than the configured absolute maximum.
            ],
            fault-status("CTRL_OVERTEMP")[
                Controller temperature higher than the configured maximum.
            ],
            fault-status("MOTOR_OVERTEMP")[
                Motor temperature higher than the configured maximum.
            ],
            fault-status("SENSOR_WIRE_FAULT")[
                Sensor differential signal fault.
            ],
            fault-status("SENSOR_GENERAL_FAULT")[
                Sensor processing fault.
            ],
            fault-status("CAN_COMMAND_ERROR")[
                Invalid CAN command received.
            ],
            fault-status("ANALOG_INPUT_ERROR")[
                Redundant sensor input out of range.
            ],
        ),
    ),
)

#let gd4-fields = (
    (
        bytes: (0, 3),
        name: "ID",
        type: "int32_t",
        desc: [$I_d$ of the FOC algorithm in hundredths of an #unit("A").],
    ),
    (
        bytes: (4, 7),
        name: "IQ",
        type: "int32_t",
        desc: [$I_q$ of the FOC algorithm in hundredths of an #unit("A").],
    ),
)

#let gd5-fields = (
    (
        byte: 0,
        name: "THROTTLE",
        type: "int8_t",
        desc: [The received throttle signal from analog inputs or #raw("CAN2") in whole #unit("percent").],
    ),
    (
        byte: 1,
        name: "BRAKE",
        type: "int8_t",
        desc: [The received brake signal from analog inputs or #raw("CAN2") in whole #unit("percent").],
    ),
    (
        byte: 2,
        name: "PINS",
        type: "uint8_t",
        desc: [The digital pin state.],
    ),
    (
        byte: 3,
        name: "DRV_EN",
        type: "bool",
        desc: [The current drive enable state.],
    ),
    (
        bytes: (4, 5),
        name: "LIMIT_FLAGS",
        type: "flags",
        desc: [The limit activation state flags, described in @dti-limit-flags.],
    ),
    (
        byte: 6,
        name: "RFU",
        type: "uint8_t",
        desc: [Reserved for future use. Must be 0xff.],
    ),
    (
        byte: 7,
        name: "CAN_MAP",
        type: "uint8_t",
        desc: [The configured #abbr.can.a map version.],
    ),
)

#let limit-flags-format = util.describe-format("DTI Limit Flags", kind: "Bitset", lbl: "dti-limit-flags", max-col-count: 4, (
    (
        bits: (15, 11),
        name: "RFU",
        desc: [Reserved for future use. Must be zero.],
    ),
    (
        bit: 10,
        name: "PWR_LIM",
    ),
    (
        bit: 9,
        name: "RPM_MAX",
    ),
    (
        bit: 8,
        name: "RPM_MIN",
    ),
    (
        bit: 7,
        name: "MOTOR_TEMP",
    ),
    (
        bit: 6,
        name: "MOTOR_ACCEL_TEMP",
    ),
    (
        bit: 5,
        name: "INP_VOLTAGE",
    ),
    (
        bit: 4,
        name: "IGBT_TEMP",
    ),
    (
        bit: 3,
        name: "IGBT_ACCEL",
    ),
    (
        bit: 2,
        name: "DRV_EN",
        desc: [Not useful, use byte in message instead.],
    ),
    (
        bit: 1,
        name: "DC_CURRENT",
    ),
    (
        bit: 0,
        name: "CAP_TEMP",
    ),
))

#util.describe-can-messages("Inverter", (
    (
        name: "General Data 1",
        id: [0x20],
        prio: [
            0
            #footnote(numbering: n => sym.dagger)[
                Because the inverter doesn't follow the exact ID format described in @can-id-format, its messages must always have priority 0.
            ] <prio-fn>
        ],
        length: [8],
        desc: [],
        fields: gd1-fields,
    ),
    (
        name: "General Data 2",
        id: [0x21],
        prio: [0 @prio-fn],
        length: [4],
        desc: [],
        fields: gd2-fields,
    ),
    (
        name: "General Data 3",
        id: [0x22],
        prio: [0 @prio-fn],
        length: [5],
        desc: [],
        fields: gd3-fields,
    ),
    (
        name: "General Data 4",
        id: [0x23],
        prio: [0 @prio-fn],
        length: [8],
        desc: [],
        fields: gd4-fields,
    ),
    (
        name: "General Data 5",
        id: [0x24],
        prio: [0 @prio-fn],
        length: [8],
        desc: [],
        fields: gd5-fields,
        after: limit-flags-format,
    ),
))
