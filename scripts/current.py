#!/usr/bin/env python3

from matplotlib.widgets import Slider
import numpy as np
import matplotlib.pyplot as plt


dt = 0.01
T = 2.0
N = int(T / dt)

fig, ax = plt.subplots()
fig.subplots_adjust(bottom=0.25)
ax_tau = fig.add_axes([0.25, 0.1, 0.65, 0.03])
tau_slider = Slider(
    ax=ax_tau,
    label='Tau',
    valmin=0.005,
    valmax=0.2,
    valinit=0.05,
)

t = np.arange(N) * dt
I = np.abs(10 * np.cos(2 * np.pi * t) + 50 * np.sin(2 * np.pi * t / 4))
I_noise = I + np.random.normal(0, 2, N)
I_filt = np.zeros(N)

ax.plot(t, I, 'k--')
ax.plot(t, I_noise, 'g-')
filt_line, = ax.plot(t, I_filt, 'b-')

charge_true = np.sum(I) * dt

def update(val):
    alpha = dt / (val + dt)
    I_filt = np.zeros(N)
    I_filt[0] = I_noise[0]
    for i in range(1, N):
        I_filt[i] = I_filt[i - 1] + alpha * (I_noise[i] - I_filt[i - 1])

    # roll_index = 0
    # rolling = np.zeros(int(val))
    # for i in range(N):
    #     rolling[i % len(rolling)] = I_noise[i]
    #     I_filt[i] = np.sum(rolling) / len(rolling)
    
    filt_line.set_ydata(I_filt)
    fig.canvas.draw_idle()

    charge_filt = np.sum(I_filt) * dt
    print(f'[{charge_true}, {charge_filt}, {charge_true - charge_filt}]')

tau_slider.on_changed(update)
plt.show()

# time = 0
# dt = 1 / 100
# alpha1 = dt / (0.05 + dt)
# alpha2 = dt / (0.01 + dt)
# Ifilt1 = 0.0
# Ifilt2 = 0.0
# while time < 5:
#     I = np.abs(np.sin(time) * 10)
#     Inoise = I + np.random.normal(0, 0.5)
#     Ifilt1 += alpha1 * (Inoise - Ifilt1)
#     Ifilt2 += alpha2 * (Inoise - Ifilt2)
#     print(f'{time:.2f}: [{I:.3f}, {Inoise:.3f}, {Ifilt1:.3f}, {Ifilt2:.3f}]')
#     time += dt
