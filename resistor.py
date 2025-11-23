#!/usr/bin/env python3

import sys

res = []
res += [3e3, 3.3e3, 3.6e3, 3.9e3, 4.3e3, 4.7e3, 5.1e3, 5.6e3, 6.2e3, 6.8e3]
res += [7.5e3, 8.2e3, 9.1e3, 10e3, 11e3, 12e3, 13e3, 15e3, 16e3, 18e3]
res += [20e3, 22e3, 24e3, 27e3, 30e3, 33e3, 36e3, 39e3, 43e3, 47e3]
res += [51e3, 56e3, 62e3, 68e3, 75e3, 82e3, 91e3, 100e3, 110e3, 120e3]
res += [130e3, 150e3, 160e3, 180e3, 200e3, 220e3, 240e2, 270e3]
res += [300e3, 330e3, 360e3, 390e3, 430e3, 470e3, 510e3, 560e3]
res += [620e3, 680e3, 750e3, 820e3, 910e3, 1e6, 1.1e6, 1.2e6, 1.3e6]
res += [1.5e6, 1.8e6, 2e6, 2.2e6, 2.4e6, 2.7e6, 3e6, 3.3e6, 3.6e6]
res += [3.9e6, 4.3e6, 4.7e6, 5.1e6]

vin = float(sys.argv[1])
target = float(sys.argv[2])

for top in res:
    for bot in res:
        vout = bot / (top + bot) * vin
        if abs(vout - target) < 0.1:
            print(f'[{top}, {bot}] = {vout}')
