#!/usr/bin/env python3

import pandas as pd
import sys

boms = [pd.read_csv(csv) for csv in sys.argv[1:]]
concatenated = pd.concat(boms, ignore_index=True)
grouped = concatenated.groupby(['Value', 'Footprint', 'Mfr', 'Mfr #'], dropna=False).agg({
    'Qty': 'sum',
    'DigiKey #': 'first',
    'Mouser #': 'first',
    'LCSC #': 'first',
})
print(grouped.to_csv())
