#import "@preview/unify:0.8.1": num,qty,numrange,qtyrange,unit

#let version = 0.1

#set document(
    title: "YFS Electronics Manual",
    author: ("Owen Smith"),
)

#set page(
    paper: "a4",
    margin: 2cm,
)

#set text(
    font: "Fira Sans",
    size: 11pt,
    region: "gb",
)

#set par(justify: true)
#show math.equation: set text(font: "Fira Math", size: 11pt)
#show raw: set text(font: "Fira Mono", size: 11pt)

// Increase default table inset which works better for math subscripts.
#set table(inset: 7pt)

// Default table headers with a background fill.
#set table(fill: (_, y) => if y == 0 { rgb("EAF2F5") })

// Title page.
#page[
    #set align(center + horizon)
    #set v(weak: true)
    #text(size: 50pt, weight: "bold")[Electronics Manual]
    #v(3em)
    #text(size: 25pt)[York Formula Student]
    #v(2.5em)
    #grid(
        columns: (35%, 35%),
        column-gutter: 10%,
        image("images/yfs_logo.png"),
        image("images/uoy_logo.png"),
    )
    #v(3em)
    #text(size: 15pt)[v#version]
    #v(2em)
    #text(size: 12pt)[
        Owen Smith \
        #link("mailto:ldp520@york.ac.uk")[
            #raw("ldp520@york.ac.uk")
        ]
    ]
]
#pagebreak()

// Front matter numbering.
#set page(numbering: "i")
#counter(page).update(1)

#outline(title: "Contents")
#pagebreak()

#outline(
    title: "List of Figures",
    target: figure.where(kind: image),
)
#pagebreak()

#outline(
    title: "List of Tables",
    target: figure.where(kind: table),
)
#pagebreak()

// Main matter.
#set page(numbering: "1")
#set heading(numbering: "1.")
#counter(page).update(1)

// Footer.
#set page(
    footer: context [
        #set text(size: 10pt, fill: gray)
        Electronics Manual --- York Formula Student --- v#version
        #h(1fr)
        Page #counter(page).display() of #counter(page).final().first()
    ]
)

#let describe-can-messages(name, messages) = [
    #figure(
        table(
            columns: (auto, auto, auto, auto, 1fr),
            align: (left, right, right, right, left),
            table.header([*Name*], [*Packet ID*], [*Default Priority*], [*Data Bytes*], [*Note*]),
            ..messages.map(m => (
                m.at("name"),
                m.at("id"),
                m.at("prio"),
                m.at("length"),
                if "note" in m { m.at("note") } else { [] },
            )).flatten(),
        ),
        caption: [#name Message Summary],
    )
]

#let map-range(field) = if "byte" in field or "bit" in field {
    num(field.at("byte", default: field.at("bit", default: "")))
} else {
    let range = field.at("bytes", default: field.at("bits", default: ""))
    numrange(range.at(0), range.at(1))
}

#let map-range-prefix(field) = if "byte" in field {
    [Byte #field.at("byte")]
} else if "bit" in field {
    [Bit #field.at("bit")]
} else if "bytes" in field {
    let range = field.at("bytes")
    [Bytes #range.at(0)-#range.at(1)]
} else {
    let range = field.at("bits")
    [Bits #range.at(0)-#range.at(1)]
}

#let range-len(field) = if "byte" in field or "bit" in field {
    // TODO: 1.1 is a hack to fix 1fr being slightly too tight in absolute terms.
    1.1
} else {
    let range = field.at("bytes", default: field.at("bits", default: ""))
    calc.abs(range.at(1) - range.at(0)) + 1
}

#let build-format-type-columns(fields) = if "type" in fields.at(0) {
  ([Type], ..fields.map(f => raw(f.at("type"))))
} else {
  ()
}

#let build-format-row(fields, max-col-count) = {
    let cell-count = 0
    let row-fields = ()
    let columns = ()
    while cell-count < max-col-count and fields.len() > 0 {
        let field = fields.remove(0)
        let field-cell-count = range-len(field)
        row-fields.push(field)
        columns.push(field-cell-count)
        cell-count += field-cell-count
    }
    return (row-fields, columns)
}

#let build-format-tables(fields, max-col-count) = {
    let tables = ()
    while fields.len() > 0 {
        let (row-fields, column-sizes) = build-format-row(fields, max-col-count)
        fields = fields.slice(row-fields.len())
        tables.push(table(
            fill: none,
            inset: 5pt,
            columns: (auto,) + column-sizes.map(l => l * 1fr),
            align: (x, y) => if x > 0 { center } else { right },
            stroke: (x, y) => if x > 0 and y > 0 { 1pt } else { 0pt },
            if "byte" in row-fields.at(0) or "bytes" in row-fields.at(0) { [Byte] } else { [Bit] },
            ..row-fields.map(map-range),
            [Field],
            ..row-fields.map(f => raw(f.at("name"))),
            ..build-format-type-columns(row-fields),
        ))
    }
    return tables
}

#let describe-format(name, fields, kind: "Message", lbl: none, max-col-count: 32) = [
    #figure(
        grid(
            row-gutter: 1em,
            ..build-format-tables(fields, max-col-count),
        ),
        caption: [#name #kind Format],
    )
    #if lbl != none {
        label(lbl)
    }
    #let list-numbering = ("",) + fields.map(map-range-prefix)
    #enum(
        numbering: i => list-numbering.at(i),
        ..fields.map(f => [
            #text(f.at("name"), font: "Fira Mono", weight: "bold")
            #if "type" in f {
                [--- #raw(f.at("type"))]
            }
            #if "desc" in f {
                linebreak()
                box(inset: (left: 1.5em))[
                    #f.at("desc")
                ]
            }
        ]),
    )
]

= System Overview

== CAN Format
The default CAN bus speed is set to #qty(500, "kb/s", per: "fraction-short").
Frames contain between #num(0) and #num(8) bytes of message data, with integer data stored in a big-endian format.
A 29-bit CAN2.0B extended identifier is used for all frames with a format described in @can-id-format.

#describe-format("CAN Frame", kind: "Extended ID", lbl: "can-id-format", (
    (
        bits: (28, 26),
        name: "PRIO",
        desc: [Frame arbitration priority where #num(0) denotes the highest priority and #num(7) the lowest.],
    ),
    (
        bits: (25, 8),
        name: "PACKET_ID",
        desc: [18-bit packet ID with a meaning specified by each node.]
    ),
    (
        bits: (7, 0),
        name: "NODE_ID",
        desc: [8-bit node ID corresponding to a component of the car.]
    ),
))

The node IDs for each component in the car are specified in @can-node-ids.

#figure(
    table(
        columns: 2,
        table.header([*Node*], [*Node ID*]),
        [Rear Distribution], [0x1],
        [Front Distribution], [0x2],
        [BMS], [0x3],
        [Precharge], [0x4],
        [DTI HV-550 Inverter], [0x5],
    ),
    caption: [CAN Node IDs],
) <can-node-ids>

= Precharge and Discharge
The precharge circuit is responsible for charging the large capacitance of the motor inverter in a controlled manner through the utilisation of a dedicated precharge resistor in order to avoid a large current spike when the AIRs close, which could weld or otherwise damage the contacts.
It has a secondary function of controlling the AIRs with a combination of the shutdown circuit on the coil's high side and software control of the coil's low side.
Additionally, the precharge board senses the AIR and precharge relay intended and actual (mechanical) states for use in the software logic and output to the TSAL's hardware logic.
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
        [$upright(V)_"LVS"$], [Low voltage input], num(9), num(12), num(16), unit("volt"),
        [$upright(V)_"ACC"$], [High voltage input], num(36), [-], num(480), unit("volt"),
        [$upright(R)_"PRE"$], [Precharge resistance], num(950), num(1000), num(1050), unit("ohm"),
        [$upright(tau)_"PRE"$], [Precharge time to #qty(95, "%") at #qty(300, "volt") & #qty(200, "uF")], [-], num(600), [-], unit("ms"),
    ),
    caption: [Precharge General Operating Conditions],
) <precharge-operating>

== CAN Messages
#describe-can-messages("Precharge", (
    (
        name: [#link(label("precharge-status-format"), "Status")],
        id: [0x1],
        prio: [1],
        length: [8],
        note: [Sent every #qty(10, "ms")],
    ),
    (
        name: [Activate],
        id: [0x2],
        prio: [3],
        length: [0],
        note: [No data, toggles precharge state],
    ),
))

=== Status Message
#describe-format("Precharge Status", lbl: "precharge-status-format", (
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
))

#describe-format("Precharge Flags", kind: "Bitset", lbl: "precharge-flags", max-col-count: 4, (
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
