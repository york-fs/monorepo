#!/usr/bin/env python3

from matplotlib.widgets import MultiCursor
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib


def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = np.linspace(-4, 4, 500)
    y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """
    import numpy as np
    from math import factorial

    try:
        window_size = np.abs(int(window_size))
        order = np.abs(int(order))
    except ValueError:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.asmatrix([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')

# dfs = []
# time_offset = 0
# for i in range(1, 6):
#     df = pd.read_csv(f'output-{i}')
#     df.iloc[:, 0] += time_offset
#     time_offset = df.iloc[:, 0].iloc[-1] + 500
#     dfs.append(df)
# big_df = pd.concat(dfs, ignore_index=True)
# big_df.to_csv('charge_1.csv', index=False)

df = pd.read_csv('charge_1.csv')

df = df[df['V1'].between(25000, 45000)]

# df = df[df['enabled'] == 1]

df['uptime'] /= 1e3

fig, axes = plt.subplots(2, 2, sharex=True)
axes = axes.flatten()

for V in range(1, 13):
    axes[0].plot(df['uptime'], df[f'V{V}'] / 1e4, label=f'cell {V}')

rolling = savitzky_golay(df['current'].to_numpy(), 201, 3)

# axs[1].plot(df['uptime'], np.abs(df['current']))
axes[2].plot(df['uptime'], np.abs(rolling))
axes[2].plot(df['uptime'], df['current_set'])
# axs[1].plot(df['uptime'], (df['state'] == 1) * 1000)

rolling2 = np.abs(df['current'].rolling(200).mean())
axes[3].plot(df['uptime'], df['current_set'])
axes[3].plot(df['uptime'], rolling2)

foo = axes[0].get_ylim()
multi = MultiCursor(None, axes, useblit=True, color='k', linestyle='--', linewidth=1)
axes[0].set_ylim(foo)

plt.show()
