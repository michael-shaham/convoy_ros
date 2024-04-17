import numpy as np


def sinusoidal_ref(T, v_min, v_max, dt):
    """
    arguments:
        T: period (s)
        v_min: min speed (m/s)
        v_max: max speed (m/s)
    """
    A = (v_max - v_min) / 2
    v_0 = (v_max + v_min) / 2
    t_range = np.arange(0, T + dt / 2, dt)

    s_ref = v_0 * t_range - A * T / (2 * np.pi) * np.sin(2 * np.pi / T * t_range)
    v_ref = -A * np.cos(2 * np.pi / T * t_range) + v_0
    a_ref = 2 * A * np.pi / T * np.sin(2 * np.pi / T * t_range)

    x_ref = np.r_[s_ref, v_ref, a_ref].reshape((3, -1))
    return x_ref, t_range
