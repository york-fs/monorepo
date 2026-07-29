#import "@preview/unify:0.8.1": num, qty
#import "util.typ"

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

= System Overview

== CAN Format
The default CAN bus speed is set to #qty(500, "kb/s", per: "fraction-short").
Frames contain between #num(0) and #num(8) bytes of message data, with integer data stored in a big-endian format.
A 29-bit CAN2.0B extended identifier is used for all frames with a format described in @can-id-format.

#util.describe-format("CAN Frame", kind: "Extended ID", lbl: "can-id-format", (
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

#include "sections/precharge.typ"
#include "sections/dti.typ"
