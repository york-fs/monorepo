#let used = state("abbrev-used", ())

#let style(short) = text(red, short)

#let s(entry, plural) = context {
    let content = (entry.short,)
    if plural {
        content.push([s])
    }
    content = style(content.join())
    if query(entry.lbl).len() != 0 { link(entry.lbl, content) } else { content }
}

#let l(entry, plural) = context {
    if not entry.symbol in used.get() {
        used.update(lst => {
            lst.push(entry.symbol)
            return lst
        })
    }
    entry.long
    if plural [s]
    sym.space.nobreak
    sym.paren.l
    sym.zwj
    s(entry, plural)
    sym.zwj
    sym.paren.r
}

#let a(entry, plural) = context if entry.symbol in used.get() { s(entry, plural) } else { l(entry, plural) }

#let make-abbr(desc) = {
    let entry = (
        "symbol": desc.at(0),
        "short": desc.at(1),
        "long": desc.at(2),
    )

    // Add label.
    entry.insert("lbl", label("abbrev-" + entry.symbol))

    // Add render functions.
    entry += (
        "s": s(entry, false),
        "l": l(entry, false),
        "a": a(entry, false),
        "pls": s(entry, true),
        "pll": l(entry, true),
        "pla": a(entry, true),
    )
    return entry
}

#let make(..entries) = entries.pos().map(e => (e.at(0): make-abbr(e))).join()

#let list(
    abbr,
    title: [List of Abbreviations],
) = {
    heading(title)

    let sorted = abbr.values().sorted(key: entry => entry.short)

    // Spread entries over two columns correctly.
    let n = calc.quo(sorted.len(), 2)
    let last = if calc.odd(sorted.len()) { sorted.remove(n) }
    sorted = sorted.slice(0, n).zip(sorted.slice(n)).flatten()
    if last != none { sorted.push(last) }
    
    let make-entry(entry) = (style[#entry.short #entry.lbl], entry.long)
    grid(
        columns: (auto, 1fr) * 2,
        inset: 5pt,
        ..sorted.map(make-entry).flatten(),
    )
}
