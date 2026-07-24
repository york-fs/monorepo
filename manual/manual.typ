#set document(
    title: "YFS Electronics Manual",
    author: ("Owen Smith"),
)

#set page(
    paper: "a4",
    margin: 2cm,
)

#set text(
    font: "Open Sans",
    size: 11pt,
    lang: "en",
    region: "GB",
)

#set par(
    justify: true,
)

#set math.equation(
    numbering: none,
)

#show raw: set text(font: "DejaVu Sans Mono", size: 11pt)

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
    #text(size: 15pt)[v0.1]
    #v(2em)
    #text(size: 12pt)[
        Owen Smith \
        #link("mailto:ldp520@york.ac.uk")[
            #raw("ldp520@york.ac.uk")
        ]
    ]
]
#pagebreak()

