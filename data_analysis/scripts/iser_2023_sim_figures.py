import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from typing import List

plt.rcParams["font.family"] = "Times New Roman"

prefix = os.path.expanduser("~") + "/cvy_ws/src"
data_loc = prefix + "/convoy_ros/convoy_data/data/"
save_file_exp = prefix + "/convoy_ros/data_analysis/figures/sim_platoon_exp.pdf"

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
    start_time = times[0] - 3.0
    end_time = times[-1] + 3.0
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


def plot_quad_l1_lfbk(quad_df, l1_df, lfbk_df):
    fig, ax = plt.subplots(2, 3, sharex="col", sharey="row", figsize=(4.7, 2.5))
    fig.subplots_adjust(0.1, 0.15, 0.89, 0.83, 0.07, 0.09)
    fig.suptitle("Platoon experiment example", fontsize=title_size)
    ax[0, 0].set_title(rf"$\|\|\cdot\|\|_2^2$ MPC", fontsize=subtitle_size)
    ax[0, 1].set_title(rf"$\|\|\cdot\|\|_1$ MPC", fontsize=subtitle_size)
    ax[0, 2].set_title("Linear feedback", fontsize=subtitle_size)

    veh_inds = [i for i in range(max(quad_df["veh_ind"]) + 1)]

    for i in veh_inds:
        quad_pfmpc_veh = quad_df[quad_df["veh_ind"] == i]
        l1_pfmpc_veh = l1_df[l1_df["veh_ind"] == i]
        lfbk_veh = lfbk_df[lfbk_df["veh_ind"] == i]

        if i > 0:
            ax[0, 0].plot(
                quad_pfmpc_veh["time"].to_numpy(),
                quad_pfmpc_veh["dist_err"].to_numpy(),
                label=f"{i}",
                zorder=max(veh_inds) - i,
                color="C{}".format(i),
                linewidth=linewidth,
                linestyle=linestyle,
                alpha=alphas[i],
            )
            ax[0, 1].plot(
                l1_pfmpc_veh["time"].to_numpy(),
                l1_pfmpc_veh["dist_err"].to_numpy(),
                label=f"{i}",
                zorder=max(veh_inds) - i,
                color="C{}".format(i),
                linewidth=linewidth,
                linestyle=linestyle,
                alpha=alphas[i],
            )
            ax[0, 2].plot(
                lfbk_veh["time"].to_numpy(),
                lfbk_veh["dist_err"].to_numpy(),
                label=f"{i}",
                zorder=max(veh_inds) - i,
                color="C{}".format(i),
                linewidth=linewidth,
                linestyle=linestyle,
                alpha=alphas[i],
            )

        # ax[0,0].plot(quad_pfmpc_veh['time'].to_numpy(),
        #              quad_pfmpc_veh['ego_position'].to_numpy(),
        #              label=f'{i}', zorder=max(veh_inds)-i,
        #              color="C{}".format(i),
        #              linewidth=linewidth, linestyle=linestyle, alpha=alphas[i])
        # ax[0,1].plot(l1_pfmpc_veh['time'].to_numpy(),
        #              l1_pfmpc_veh['ego_position'].to_numpy(),
        #              label=f'{i}', zorder=max(veh_inds)-i,
        #              color="C{}".format(i),
        #              linewidth=linewidth, linestyle=linestyle, alpha=alphas[i])
        # ax[0,2].plot(lfbk_veh['time'].to_numpy(),
        #              lfbk_veh['ego_position'].to_numpy(),
        #              label=f'{i}', zorder=max(veh_inds)-i,
        #              color="C{}".format(i),
        #              linewidth=linewidth, linestyle=linestyle, alpha=alphas[i])

        ax[1, 0].plot(
            quad_pfmpc_veh["time"].to_numpy(),
            quad_pfmpc_veh["ego_speed"].to_numpy(),
            label=f"{i}",
            zorder=max(veh_inds) - i,
            color="C{}".format(i),
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[i],
        )
        ax[1, 1].plot(
            l1_pfmpc_veh["time"].to_numpy(),
            l1_pfmpc_veh["ego_speed"].to_numpy(),
            label=f"{i}",
            zorder=max(veh_inds) - i,
            color="C{}".format(i),
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[i],
        )
        ax[1, 2].plot(
            lfbk_veh["time"].to_numpy(),
            lfbk_veh["ego_speed"].to_numpy(),
            label=f"{i}",
            zorder=max(veh_inds) - i,
            color="C{}".format(i),
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[i],
        )

    ax[0, 0].set_ylabel("position [m]", fontsize=axes_size)
    ax[1, 0].set_ylabel("velocity [m/s]", fontsize=axes_size)
    ax[1, 0].set_xlabel("time [s]", fontsize=axes_size)
    ax[1, 1].set_xlabel("time [s]", fontsize=axes_size)
    ax[1, 2].set_xlabel("time [s]", fontsize=axes_size)

    for a in ax.flat:
        a.grid()
        a.tick_params(axis="both", labelsize=tick_label_size)
    for a in ax[1, :]:
        a.set_yticks([0.0, 2.0, 3.5])
    ax[0, 2].legend(
        bbox_to_anchor=(0.99, 0.5),
        loc="center left",
        title="vehicle",
        title_fontsize=legend_title_size,
        fontsize=legend_font_size,
    )
    ax[1, 2].legend(
        bbox_to_anchor=(0.99, 0.5),
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
    lfbk_files = [file for file in data_files if "sim_lfbk_0" in file]
    quad_pfmpc_files = [file for file in data_files if "sim_quad_pfmpc_0" in file]
    l1_pfmpc_files = [file for file in data_files if "sim_l1_pfmpc_0" in file]

    lfbk_df = pd.concat([pd.read_csv(data_loc + file) for file in lfbk_files])
    lfbk_df = remove_zero_vel(lfbk_df)
    lfbk_df = add_position_column(lfbk_df)

    quad_pfmpc_df = pd.concat(
        [pd.read_csv(data_loc + file) for file in quad_pfmpc_files]
    )
    quad_pfmpc_df = remove_zero_vel(quad_pfmpc_df)
    quad_pfmpc_df = add_position_column(quad_pfmpc_df)

    l1_pfmpc_df = pd.concat([pd.read_csv(data_loc + file) for file in l1_pfmpc_files])
    l1_pfmpc_df = remove_zero_vel(l1_pfmpc_df)
    l1_pfmpc_df = add_position_column(l1_pfmpc_df)

    plot_quad_l1_lfbk(quad_pfmpc_df, l1_pfmpc_df, lfbk_df)
