import numpy as np


class LinearFeedback:
    """
    Platoon linear feedback controller.

    Parameters:
        position_gain: positive float, gain that multiplies position error
        speed_gain: positive float, gain that multiplies speed error
    """

    def __init__(self, position_gain, speed_gain):
        self.k_p = position_gain
        self.k_s = speed_gain

    def control(self, position_error, speed_error):
        return -(self.k_p * position_error + self.k_s * speed_error)
