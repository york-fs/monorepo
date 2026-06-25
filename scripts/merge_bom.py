#!/usr/bin/env python3

import csv
import pandas as pd
import sys


def assert_unique(series):
    unique = series.unique()
    if len(unique) > 1:
        values = ', '.join(str(x) for x in unique)
        raise ValueError(f'Multiple values in column "{series.name}": "{values}"')
    return unique[0] if len(unique) == 1 else None


def concatenate_references(series):
    if len(series.unique()) != len(series):
        raise ValueError('Duplicate designators')
    return ','.join(str(x) for x in series)


boms = [pd.read_csv(csv) for csv in sys.argv[1:]]
concatenated = pd.concat(boms, ignore_index=True)

# Add quantity column with each row (BOM item) initialised to 1.
concatenated['Qty'] = 1

# Check that no entries have leading or trailing whitespace which can easily happen in KiCad's BOM UI.
for column in ['Mfr', 'Mfr #', 'DigiKey #', 'Mouser #', 'LCSC #']:
    filtered = concatenated[column].dropna().loc[lambda x: x != x.str.strip()]
    if not filtered.empty:
        raise ValueError(f'Extraneous whitespace in column "{column}"')

agg_list = {
    'DigiKey #': assert_unique,
    'Mouser #': assert_unique,
    'LCSC #': assert_unique,
    'Qty': 'sum',
}
if len(boms) == 1:
    agg_list['Reference'] = concatenate_references
grouped = concatenated.groupby(['Value', 'Footprint', 'Mfr', 'Mfr #', 'DNP'], dropna=False, as_index=False, sort=False).agg(agg_list)
column_order = ['Reference', 'Value', 'Footprint', 'Qty', 'Mfr', 'Mfr #', 'DigiKey #', 'Mouser #', 'LCSC #', 'DNP']
if len(boms) != 1:
    column_order = column_order[1:]
reordered = grouped.loc[:, column_order]
print(reordered.to_csv(index=False, quoting=csv.QUOTE_ALL))
