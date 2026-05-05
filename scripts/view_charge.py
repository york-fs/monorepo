#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import sys


df = pd.read_csv(sys.argv[1])
df = df[df['V1'].between(25000, 45000)]
df['uptime'] /= 1e3

mode = sys.argv[2]
if mode == 'voltages':
    for V in range(1, 13):
        plt.plot(df['uptime'], df[f'V{V}'] / 1e4, label=f'Cell {V}', color=plt.cm.tab20.colors[V - 1])
    plt.title('Plot of Per-Cell Voltages Against Charging Time')
    plt.xlabel('Time / s')
    plt.ylabel('Voltage / V')
    plt.legend()
    plt.grid()
    plt.show()
elif mode == 'temperature':
    df = df[df['balance_temp_1'] > 0]
    df = df[df['balance_temp_2'] > 0]
    df = df[df['balance_temp_3'] > 0]
    plt.plot(df['uptime'], df['balance_temp_1'], label='Balance Temperature 1')
    plt.plot(df['uptime'], df['balance_temp_2'], label='Balance Temperature 2')
    plt.plot(df['uptime'], df['balance_temp_3'], label='Balance Temperature 3')
    plt.legend()
    plt.grid()
    plt.show()
elif mode == '3d':
    voltages = [df[f'V{V}'] / 1e4 for V in range(1, 13)]
    V = np.vstack(voltages)
    t = np.array(df['uptime'])
    channels = np.arange(12)
    fig = go.Figure(data=[go.Surface(z=V, x=t, y=channels, colorscale='Viridis')])
    fig.update_layout(
        scene=dict(
            xaxis_title='Time',
            yaxis_title='Cell',
            zaxis_title='Voltage'
        )
    )
    fig.show()
