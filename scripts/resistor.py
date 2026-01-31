#!/usr/bin/env python3

import argparse
import itertools
import sys


# Add E24 resistor series up to 10M.
res = []
e24 = [1, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2, 2.2, 2.4, 2.7, 3, 3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1]
for scale in (10**x for x in range(7)):
    res += [x * scale for x in e24]
res += [10e6]

args_parser = argparse.ArgumentParser(prog=sys.argv[0], description='Resistive divider calculator')
args_parser.add_argument('-c', '--count', type=int, default=10, help='number of results to show')
args_parser.add_argument('-p', '--power', type=float, default=62.5, help='maximum resistor power loss in milliwatts')
args_parser.add_argument('-r', '--impedance', type=float, default=500, help='maximum output impedance in kiloohms')
args_parser.add_argument('input', type=float, help='input voltage')
args_parser.add_argument('target', type=float, help='desired output voltage')
args = args_parser.parse_args()

options = set()
tolerance = 0.01
while tolerance <= abs(args.input - args.target) / 10:
    for top, bot in itertools.product(res, res):
        I = args.input / (top + bot)
        Ptop = I**2 * top * 1e3
        Pbot = I**2 * bot * 1e3

        # Check that neither resistor exceeds the power limit.
        if Ptop > args.power or Pbot > args.power:
            continue

        # Check that the output impedance is below the maximum allowable.
        Z = 1e-3 / (1/top + 1/bot)
        if Z > args.impedance:
            continue

        # Check that the output voltage is within tolerance.
        ratio = bot / (top + bot)
        Vout = args.input * ratio
        error = abs(Vout - args.target)
        if error > tolerance:
            continue
        options.add((top, bot, Vout, ratio, I, Z, Ptop, Pbot))
    tolerance *= 2

def key(result):
    error = abs(result[2] - args.target)
    return error + result[4]/2 + result[5]/1e4
results = sorted(options, key=key)[:args.count]

print(f'Found {len(options)} candidates for Vin={args.input}V, Vout={args.target}V, Pmax={args.power}mW, Zmax={args.impedance}kΩ')
print('(    Rt/Ω,     Rb/Ω) = (Vout/V, Ratio, I/mA,  Z/kΩ, Pt/mW, Pb/mW)')
for top, bot, Vout, ratio, I, Z, Ptop, Pbot in results:
    print(f'({top:8.8g}, {bot:8.8g}) = ({Vout:6.2f}, {1/ratio:5.2f}, {I*1e3:4.1f}, {Z:5.1f}, {Ptop:5.1f}, {Pbot:5.1f})')
