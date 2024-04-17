"""
Visualizes the position and velocity data from a single trial logged using the 
convoy_data logger in log_data.py.
"""

import matplotlib.pyplot as plt
import os
import pandas as pd
import sys

plt.rcParams["font.family"] = "Times New Roman"

prefix = os.path.expanduser("~") + "/cvy_ws/src/convoy_ros"
if sys.platform == "darwin":
    prefix = os.path.expanduser("~") + "/projects/convoy_ros"
data_loc = prefix + "/convoy_data/data/"

title_size = 10
subtitle_size = 8
axes_size = 7
legend_title_size = 8
legend_font_size = 7
tick_label_size = 7
linewidth = 0.8
linestyle = "solid"
alphas = [0.7, 0.8, 0.8, 0.8]
scatter_size = 6


def remove_zero_vel(df: pd.DataFrame):
    lead_veh_df = df[df["veh_ind"] == 0]
    times = lead_veh_df[lead_veh_df["pred_speed"] > 0]["time"].to_numpy()
    start_time = times[0] - 2.0
    end_time = times[-1] + 2.0
    df = df[df["time"] > start_time]
    df = df[df["time"] < end_time]
    df["time"] -= start_time
    return df


def add_position_column(df: pd.DataFrame):
    veh_ind = df["veh_ind"].to_numpy()
    time = df["time"].to_numpy()
    speed = df["ego_speed"].to_numpy()
    dist = df["pred_dist"].to_numpy()
    position = []  # we will fill this out for each row in df

    # initialize vehicle positions and get init time for veh 0
    n_vehicles = max(veh_ind) + 1
    veh_pos = [0.0]
    for k in range(0, n_vehicles):
        if k == 0:
            for i, t in enumerate(time):
                if veh_ind[i] == k:
                    prev_time = time[i]
                    break
        else:
            for i, d in enumerate(dist):
                if veh_ind[i] == k:
                    veh_pos.append(veh_pos[k - 1] - d)
                    break

    for k, ind in enumerate(veh_ind):
        if ind == 0:
            dt = time[k] - prev_time
            veh_pos[ind] += dt * speed[k]
            prev_time = time[k]
            position.append(veh_pos[ind])
        else:
            veh_pos[ind] = veh_pos[ind - 1] - dist[k]
            position.append(veh_pos[ind])
    df["ego_position"] = position
    return df


def plot_trial(df: pd.DataFrame):
    veh_inds = [i for i in range(max(df["veh_ind"]) + 1)]

    fig, ax = plt.subplots(2, 1, sharex="col", sharey="row", figsize=(3.0, 2.5))
    fig.subplots_adjust(0.1, 0.15, 0.89, 0.83, 0.07, 0.09)
    fig.suptitle(
        f"Platoon trajectories: Hardware comparison with {max(veh_inds) + 1} vehicles",
        fontsize=title_size,
    )
    ax[0].set_title(rf"$\|\|\cdot\|\|_2^2$ DMPC", fontsize=subtitle_size)
    ax[0].set_title(rf"$\|\|\cdot\|\|_1$ DMPC", fontsize=subtitle_size)
    ax[0].set_title("Linear feedback", fontsize=subtitle_size)

    for i in veh_inds:
        veh_data = df[df["veh_ind"] == i]

        ax[0].plot(
            veh_data["time"].to_numpy(),
            veh_data["ego_position"].to_numpy(),
            label=f"{i}",
            zorder=max(veh_inds) - i,
            color="C{}".format(i),
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[i],
        )
        ax[1].plot(
            veh_data["time"].to_numpy(),
            veh_data["ego_speed"].to_numpy(),
            label=f"{i}",
            zorder=max(veh_inds) - i,
            color="C{}".format(i),
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[i],
        )

    ax[0].set_ylabel("position [m]", fontsize=axes_size)
    ax[1].set_ylabel("velocity [m/s]", fontsize=axes_size)
    ax[1].set_xlabel("time [s]", fontsize=axes_size)

    for a in ax.flat:
        a.grid()
        a.tick_params(axis="both", labelsize=tick_label_size)
    ax[1].set_yticks([0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
    ax[1].legend(
        bbox_to_anchor=(0.99, 1.1),
        loc="center left",
        title="vehicle",
        title_fontsize=legend_title_size,
        fontsize=legend_font_size,
    )
    plt.show()
    fig.savefig(save_file_exp, bbox_inches="tight")


if __name__ == "__main__":
    data_files = os.listdir(data_loc)

    data_files.sort()
    trial_name = input("Enter trial name without extension: ")
    file = [file for file in data_files if trial_name in file][0]
    save_file_exp = prefix + f"/data_analysis/figures/{trial_name}_traj.pdf"

    df = pd.read_csv(data_loc + file)
    df = remove_zero_vel(df)
    df = add_position_column(df)

    plot_trial(df)
