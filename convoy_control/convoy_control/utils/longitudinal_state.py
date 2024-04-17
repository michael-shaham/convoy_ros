import numpy as np

from convoy_interfaces.msg import LongitudinalState


def create_long_state_trajectory(x_traj: np.ndarray):
    states = []
    for x in x_traj.T:
        state = LongitudinalState()
        state.position = x[0]
        state.velocity = x[1]
        state.acceleration = x[2]
        states.append(state)
    return states


def create_long_state_np(states_msgs: list):
    """states_msgs is a list of LongitudinalState msgs."""
    x = []
    for state in states_msgs:
        x.append([state.position, state.velocity, state.acceleration])
    return np.array(x)
