import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from scipy.integrate import ode

plt.rcParams["font.family"] = "Times New Roman"


def forward_discrete(v, v_des, dt, tau):
    return (1 - dt / tau) * v + dt / tau * v_des


def forward_continuous(t, y, tau, y_des):
    return -1 / tau * y + 1 / tau * y_des


if __name__ == "__main__":
    data_loc = os.path.expanduser("~") + "/cvy_ws/src/convoy_ros/data_analysis/data/"
    data_files = os.listdir(data_loc)
    data_files.sort()
    data_files = [file for file in data_files if ".gitignore" not in file]
    data_files = [file for file in data_files if "stagger" in file]
    fig, ax = plt.subplots(len(data_files))
    for i, file in enumerate(data_files):
        print(file)
        if ".csv" not in file:
            continue
        df = pd.read_csv(data_loc + file)
        df = df[["time", "speed", "desired speed"]]
        # df.to_csv(data_loc + file)

        tau = 0.1

        dt_discrete = 0.02
        time_discrete = np.arange(0.0, df["time"].to_numpy()[-1], dt_discrete)

        dt_cont = 0.02
        time_cont = np.arange(0.0, df["time"].to_numpy()[-1], dt_cont)

        desired_speed = df["desired speed"].to_numpy()

        v_discrete = np.zeros_like(time_discrete)
        v_cont = np.zeros_like(time_cont)

        r = ode(forward_continuous)
        r.set_initial_value(0, 0)

        for i in range(len(v_discrete) - 1):
            v_discrete[i + 1] = forward_discrete(
                v_discrete[i], desired_speed[i], dt_discrete, tau
            )

        for i in range(len(v_cont) - 1):
            r.set_f_params(tau, desired_speed[i])
            v_cont[i + 1] = r.integrate(r.t + dt_cont)

        ax.plot(df["time"].to_numpy(), df["speed"].to_numpy(), label="speed")
        ax.plot(
            df["time"].to_numpy(), df["desired speed"].to_numpy(), label="desired speed"
        )
        ax.plot(time_discrete, v_discrete, label="discrete model")
        ax.plot(time_cont, v_cont, label="continuous model")
        ax.legend(bbox_to_anchor=(1.01, 0.5), loc="center left")
        ax.grid()
    plt.show()
