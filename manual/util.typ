#import "@preview/unify:0.8.1": num,qty,numrange,qtyrange,unit

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
