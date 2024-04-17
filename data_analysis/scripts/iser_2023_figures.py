import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import sys
from typing import List

plt.rcParams["font.family"] = "Times New Roman"

prefix = os.path.expanduser("~") + "/cvy_ws/src/convoy_ros"
if sys.platform == "darwin":
    prefix = os.path.expanduser("~") + "/projects/convoy_ros"
data_loc = prefix + "/convoy_data/data/"
save_file_exp = prefix + "/data_analysis/figures/iser_platoon_exp.pdf"
save_file_rms = prefix + "/data_analysis/figures/iser_rms_fig.pdf"

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


def plot_quad_l1_lfbk(quad_df, l1_df, lfbk_df):
    veh_inds = [i for i in range(max(quad_df["veh_ind"]) + 1)]

    fig, ax = plt.subplots(2, 3, sharex="col", sharey="row", figsize=(4.7, 2.5))
    fig.subplots_adjust(0.1, 0.15, 0.89, 0.83, 0.07, 0.09)
    fig.suptitle(
        f"Platoon trajectories: Hardware comparison with {max(veh_inds) + 1} vehicles",
        fontsize=title_size,
    )
    ax[0, 0].set_title(rf"$\|\|\cdot\|\|_2^2$ DMPC", fontsize=subtitle_size)
    ax[0, 1].set_title(rf"$\|\|\cdot\|\|_1$ DMPC", fontsize=subtitle_size)
    ax[0, 2].set_title("Linear feedback", fontsize=subtitle_size)

    for i in veh_inds:
        quad_pfmpc_veh = quad_df[quad_df["veh_ind"] == i]
        l1_pfmpc_veh = l1_df[l1_df["veh_ind"] == i]
        lfbk_veh = lfbk_df[lfbk_df["veh_ind"] == i]

        ax[0, 0].plot(
            quad_pfmpc_veh["time"].to_numpy(),
            quad_pfmpc_veh["ego_position"].to_numpy(),
            label=f"{i}",
            zorder=max(veh_inds) - i,
            color="C{}".format(i),
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[i],
        )
        ax[0, 1].plot(
            l1_pfmpc_veh["time"].to_numpy(),
            l1_pfmpc_veh["ego_position"].to_numpy(),
            label=f"{i}",
            zorder=max(veh_inds) - i,
            color="C{}".format(i),
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[i],
        )
        ax[0, 2].plot(
            lfbk_veh["time"].to_numpy(),
            lfbk_veh["ego_position"].to_numpy(),
            label=f"{i}",
            zorder=max(veh_inds) - i,
            color="C{}".format(i),
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[i],
        )

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
    ax[1, 2].legend(
        bbox_to_anchor=(0.99, 1.1),
        loc="center left",
        title="vehicle",
        title_fontsize=legend_title_size,
        fontsize=legend_font_size,
    )
    plt.show()
    fig.savefig(save_file_exp, bbox_inches="tight")


def get_rms_errors(dfs: List[pd.DataFrame]):
    veh_inds = [i for i in range(1, max(dfs[0]["veh_ind"]) + 1)]
    n_vehicles = len(veh_inds)
    dist_rms_errors = [[] for _ in veh_inds]
    speed_rms_errors = [[] for _ in veh_inds]

    for df in dfs:
        for k, i in enumerate(veh_inds):
            veh_df = df[df["veh_ind"] == i]
            dist_err = veh_df["dist_err"].to_numpy()
            speed_err = veh_df["speed_err"].to_numpy()

            dist_rms_errors[k].append(
                np.sqrt(np.sum(np.square(dist_err)) / len(dist_err))
            )
            speed_rms_errors[k].append(
                np.sqrt(np.sum(np.square(speed_err)) / len(speed_err))
            )

    dist_rms_means = np.array([np.mean(dist_rms_errors[i]) for i in range(n_vehicles)])
    dist_rms_stds = np.array(
        [np.std(dist_rms_errors[i], ddof=1) for i in range(n_vehicles)]
    )
    speed_rms_means = np.array(
        [np.mean(speed_rms_errors[i]) for i in range(n_vehicles)]
    )
    speed_rms_stds = np.array(
        [np.std(speed_rms_errors[i], ddof=1) for i in range(n_vehicles)]
    )

    return (
        dist_rms_errors,
        speed_rms_errors,
        dist_rms_means,
        dist_rms_stds,
        speed_rms_means,
        speed_rms_stds,
    )


def plot_mean_std_rms_error(
    df_list_of_lists: List[List[pd.DataFrame]], method_names: List[str]
):
    assert len(df_list_of_lists) == len(method_names)

    fig, ax = plt.subplots(1, 2, sharey="row", figsize=(4.7, 1.75))
    fig.subplots_adjust(0.09, 0.23, 0.77, 0.79, 0.08, 0.1)
    fig.suptitle("Mean RMSE with 95% confidence intervals", fontsize=title_size)
    ax[0].set_title("Spacing error", fontsize=subtitle_size)
    ax[1].set_title("Velocity error", fontsize=subtitle_size)

    for i, method in enumerate(method_names):
        rms_results = get_rms_errors(df_list_of_lists[i])
        dist_rms_errors, speed_rms_errors = rms_results[:2]
        dist_rms_means, dist_rms_stds, speed_rms_means, speed_rms_stds = rms_results[2:]
        veh_inds = range(1, len(dist_rms_means) + 1)
        # for j in range(len(veh_inds)):
        #     ax[0].scatter((j+1)*np.ones(len(dist_rms_errors[j])), dist_rms_errors[j],
        #                 color="C{}".format(i), s=scatter_size)
        #     ax[1].scatter((j+1)*np.ones(len(speed_rms_errors[j])), speed_rms_errors[j],
        #                 color="C{}".format(i), s=scatter_size)
        ax[0].plot(veh_inds, dist_rms_means, color="C{}".format(i), label=method)
        ax[0].fill_between(
            veh_inds,
            dist_rms_means - dist_rms_stds / np.sqrt(10),
            dist_rms_means + dist_rms_stds / np.sqrt(10),
            alpha=0.4,
        )
        ax[1].plot(veh_inds, speed_rms_means, color="C{}".format(i), label=method)
        ax[1].fill_between(
            veh_inds,
            speed_rms_means - speed_rms_stds / np.sqrt(10),
            speed_rms_means + speed_rms_stds / np.sqrt(10),
            alpha=0.4,
        )

    ax[1].legend(
        bbox_to_anchor=(1.0, 0.5),
        loc="center left",
        fontsize=legend_font_size,
        title="Method",
        title_fontsize=legend_title_size,
    )

    ax[0].set_ylabel("RMSE", fontsize=axes_size)
    ax[0].set_xlabel("Vehicle index", fontsize=axes_size)
    ax[1].set_xlabel("Vehicle index", fontsize=axes_size)

    for a in ax:
        a.grid()
        a.set_xticks([1, 2, 3])
        a.tick_params(axis="both", labelsize=tick_label_size)

    plt.show()
    fig.savefig(save_file_rms, bbox_inches="tight")


if __name__ == "__main__":
    data_files = os.listdir(data_loc)
    data_files.sort()
    quad_pfmpc_files = [file for file in data_files if "quad_pfmpc" in file]
    l1_pfmpc_files = [file for file in data_files if "l1_pfmpc" in file]
    lfbk_files = [file for file in data_files if "lfbk" in file]

    lfbk_dfs = []
    quad_pfmpc_dfs = []
    l1_pfmpc_dfs = []
    for file in lfbk_files:
        df = pd.read_csv(data_loc + file)
        df = remove_zero_vel(df)
        df = add_position_column(df)
        lfbk_dfs.append(df)
    for file in quad_pfmpc_files:
        df = pd.read_csv(data_loc + file)
        df = remove_zero_vel(df)
        df = add_position_column(df)
        quad_pfmpc_dfs.append(df)
    for file in l1_pfmpc_files:
        df = pd.read_csv(data_loc + file)
        df = remove_zero_vel(df)
        df = add_position_column(df)
        l1_pfmpc_dfs.append(df)

    lfbk_plot_ind = 2
    quad_pfmpc_plot_ind = 3
    l1_pfmpc_plot_ind = 7

    quad_pfmpc_plot_df = quad_pfmpc_dfs[quad_pfmpc_plot_ind]
    l1_pfmpc_plot_df = l1_pfmpc_dfs[l1_pfmpc_plot_ind]
    lfbk_plot_df = lfbk_dfs[lfbk_plot_ind]

    plot_quad_l1_lfbk(quad_pfmpc_plot_df, l1_pfmpc_plot_df, lfbk_plot_df)
    plot_mean_std_rms_error(
        [quad_pfmpc_dfs, l1_pfmpc_dfs, lfbk_dfs],
        method_names=[
            rf"$\|\|\cdot\|\|_2^2$ DMPC",
            "$\|\|\cdot\|\|_1$ DMPC",
            "Linear feedback",
        ],
    )
