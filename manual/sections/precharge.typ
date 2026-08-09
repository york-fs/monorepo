#import "@preview/unify:0.8.1": num, qty, numrange, unit
#import "../abbrevs.typ": abbr
#import "../util.typ"

= Precharge and Discharge
The precharge circuit is responsible for charging the large capacitance of the motor inverter in a controlled manner through the utilisation of a dedicated precharge resistor in order to avoid a large current spike when the #abbr.air.pla close, which could weld or otherwise damage the contacts.
It has a secondary function of controlling the #abbr.air.pla with a combination of the shutdown circuit on the coil's high side and software control of the coil's low side.
Additionally, the precharge board senses the #abbr.air.a and precharge relay intended and actual (mechanical) states for use in the software logic and output to the #abbr.tsal.a hardware logic.
The key specifications of the precharge circuit are summarised in @precharge-maximum and @precharge-operating.

#figure(
    table(
        columns: (auto, 3fr,) + (1fr,) * 3,
        align: (center, left, right, right, center),
        table.header([*Symbol*], [*Specification*], [*Min*], [*Max*], [*Unit*]),
        [$upright(V)_"LVS"$], [Low voltage input], num(-0.3), num(16), unit("volt"),
        [$upright(V)_"ACC"$], [High voltage input], num(-95), num(600), unit("volt"),
    ),
    caption: [Precharge Absolute Maximum Ratings],
) <precharge-maximum>

#figure(
    table(
        columns: (auto, 3fr,) + (1fr,) * 4,
        align: (center, left, right, right, right, center),
        table.header([*Symbol*], [*Specification*], [*Min*], [*Typ*], [*Max*], [*Unit*]),
        [$upright(V)_"LVS"$], [Low voltage input], num(9), num(12), num(15), unit("volt"),
        [$upright(V)_"ACC"$], [High voltage input], num(36), [-], num(480), unit("volt"),
        [$upright(R)_"PRE"$], [Precharge resistance], num(950), num(1000), num(1050), unit("ohm"),
        [$upright(tau)_"PRE"$], [Precharge time to #qty(95, "%") at #qty(300, "volt") & #qty(200, "uF")], [-], num(600), [-], unit("ms"),
    ),
    caption: [Precharge General Operating Conditions],
) <precharge-operating>

== CAN Messages
#let status-message-fields = (
    (
        bytes: (0, 1),
        name: "PRCHG_VOLTAGE",
        type: "uint16_t",
        desc: [The measured voltage directly after the precharge relay in #unit("volt").],
    ),
    (
        bytes: (2, 3),
        name: "TS_VOLTAGE",
        type: "uint16_t",
        desc: [The measured voltage directly after the precharge resistor in #unit("volt").]
    ),
    (
        bytes: (4, 5),
        name: "LAST_FLAGS",
        type: "flags",
        desc: [The error flags of the last attempted precharge, described in @precharge-flags.],
    ),
    (
        byte: 6,
        name: "STATE",
        type: "enum",
        desc: enum(
            numbering: i => [#(i - 1):],
            raw("LED_CHECK"),
            raw("PRECHECK"),
            raw("STANDBY"),
            raw("PRECHARGE"),
            raw("PRECHARGE_HOLD"),
            raw("ACTIVE"),
        ),
    ),
    (
        byte: 7,
        name: "MCU_TEMP",
        type: "int8_t",
        desc: [Temperature measured from the #raw("STM32F103") in #unit("dC").],
    ),
)

#let flags-format = util.describe-format("Precharge Flags", kind: "Bitset", lbl: "precharge-flags", max-col-count: 4, (
    (
        bits: (15, 12),
        name: "RFU",
        desc: [Reserved for future use. Must be zero.],
    ),
    (
        bit: 11,
        name: "DEACTIVATED",
        desc: [The system was deactivated by CAN request in the #raw("ACTIVE") state.],
    ),
    (
        bit: 10,
        name: "AIR_NEG_OPEN",
        desc: [The negative AIR was measured as open when it should have been closed in the #raw("PRECHARGE"), #raw("PRECHARGE_HOLD"), or #raw("ACTIVE") state.],
    ),
    (
        bit: 9,
        name: "AIR_POS_OPEN",
        desc: [The positive AIR was measured as open when it should have been closed in the #raw("PRECHARGE_HOLD") or #raw("ACTIVE") states.],
    ),
    (
        bit: 8,
        name: "PRCHG_OPEN",
        desc: [The precharge relay was measured as open when it should have been closed in the #raw("PRECHARGE") or #raw("PRECHARGE_HOLD") state.],
    ),
    (
        bit: 7,
        name: "SDN_OPEN",
        desc: [The shutdown circuit was measured as open during the #raw("PRECHARGE"), #raw("PRECHARGE_HOLD"), or #raw("ACTIVE") state.],
    ),
    (
        bit: 6,
        name: "WAIT_ACT",
        desc: [Awaiting an activate request over CAN in the #raw("STANDBY") state.],
    ),
    (
        bit: 5,
        name: "WAIT_DISCHG",
        desc: [Undischarged voltage was measured at the tractive system in the #raw("PRECHECK") or #raw("STANDBY") state.],
    ),
    (
        bit: 4,
        name: "PRCHK_VOLTAGE",
        desc: [Excessive voltage was measured on the output of the precharge relay in the #raw("PRECHECK") or #raw("STANDBY") state.],
    ),
    (
        bit: 3,
        name: "AIR_NEG_CLOSED",
        desc: [The negative AIR was measured as closed when it should have been open in the #raw("PRECHECK") or #raw("STANDBY") state.],
    ),
    (
        bit: 2,
        name: "AIR_POS_CLOSED",
        desc: [The positive AIR was measured as closed when it should have been open in the #raw("PRECHECK"), #raw("STANDBY"), or #raw("PRECHARGE") state.],
    ),
    (
        bit: 1,
        name: "PRCHG_CLOSED",
        desc: [The precharge relay was measured as closed when it should have been open in the #raw("PRECHECK"), #raw("STANDBY"), or #raw("ACTIVE") state.],
    ),
    (
        bit: 0,
        name: "DISCHG_OPEN",
        desc: [The shutdown circuit was measured as closed, meaning the discharge relay was assumed to be open when it should have been closed in the #raw("PRECHECK") or #raw("STANDBY") state.],
    ),
))

#util.describe-can-messages("Precharge", (
    (
        name: "Status",
        id: [0x1],
        prio: [1],
        length: [8],
        desc: [
            This message contains continuous status updates of the precharge, which is sent every #qty(10, "ms").
        ],
        fields: status-message-fields,
        after: flags-format,
    ),
    (
        name: "Activate",
        id: [0x2],
        prio: [3],
        length: [0],
        desc: [
            This message contains no data but attempts a toggle of the precharge activation state when received.
        ],
    ),
))
