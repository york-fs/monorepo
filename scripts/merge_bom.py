#!/usr/bin/env python3

import pandas as pd
import sys

boms = [pd.read_csv(csv) for csv in sys.argv[1:]]
concatenated = pd.concat(boms, ignore_index=True)

merged = (
    concatenated.groupby(["Value", "Footprint", "Mfr", "Mfr #"], dropna=False)
    .agg({
        "Qty": "sum",
        "Dist": lambda x: ", ".join(sorted(set(x.dropna()))),
        "Dist #": lambda x: ", ".join(sorted(set(x.dropna()))),
    })
)

print(merged.to_csv())
