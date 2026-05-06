#!/usr/bin/env python3

import pandas as pd
import sys


def assert_unique(series):
    unique = series.dropna().unique()
    if len(unique) > 1:
        values = ', '.join(unique)
        raise ValueError(f'Multiple values in column "{series.name}": "{values}"')
    return unique[0] if len(unique) == 1 else None


boms = [pd.read_csv(csv) for csv in sys.argv[1:]]
concatenated = pd.concat(boms, ignore_index=True)

# Check that no entries have leading or trailing whitespace which can easily happen in KiCad's BOM UI.
for column in ['Mfr', 'Mfr #', 'DigiKey #', 'Mouser #', 'LCSC #']:
    filtered = concatenated[column].dropna().loc[lambda x: x != x.str.strip()]
    if not filtered.empty:
        raise ValueError(f'Extraneous whitespace in column "{column}"')

grouped = concatenated.groupby(['Value', 'Footprint', 'Mfr', 'Mfr #'], dropna=False).agg({
    'DigiKey #': assert_unique,
    'Mouser #': assert_unique,
    'LCSC #': assert_unique,
    'Qty': 'sum',
})
print(grouped.to_csv())
