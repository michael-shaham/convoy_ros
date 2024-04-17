import matplotlib.pyplot as plt
import os
import pandas as pd
import sys

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["pdf.fonttype"] = 42

prefix = os.path.expanduser("~") + "/cvy_ws/src/convoy_ros"
if sys.platform == "darwin":
    prefix = os.path.expanduser("~") + "/Documents/convoy_ros"
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


if __name__ == "__main__":
    data_files = os.listdir(data_loc)

    data_files.sort()
    cth_trial_name = "dmpc_cth"
    cdh_trial_name = "dmpc_cdh"

    cth_files = [f for f in data_files if cth_trial_name in f]
    cdh_files = [f for f in data_files if cdh_trial_name in f]

    cdh_ind = 1
    cth_ind = 1
    cth_df = pd.read_csv(data_loc + cth_files[cth_ind])
    cth_df = remove_zero_vel(cth_df)
    cth_df = add_position_column(cth_df)
    cdh_df = pd.read_csv(data_loc + cdh_files[cdh_ind])
    cdh_df = remove_zero_vel(cdh_df)
    cdh_df = add_position_column(cdh_df)

    veh_inds = [i for i in range(max(cth_df["veh_ind"]) + 1)]
    fig, ax = plt.subplots(4, 2, sharex=True, sharey="row", figsize=(3.3, 3.6))
    fig.subplots_adjust(.12, .10, .86, .95, .07, .1)
    ax[0, 0].set_title("CTH", fontsize=subtitle_size)
    ax[0, 1].set_title("CDH", fontsize=subtitle_size)
    for i in veh_inds:
        cth_veh_data = cth_df[cth_df["veh_ind"] == i]
        cdh_veh_data = cdh_df[cdh_df["veh_ind"] == i]
        veh_data = [cth_veh_data, cdh_veh_data]
        for j, data in enumerate(veh_data):
            ax[0, j].plot(
                data["time"].to_numpy(),
                data["ego_position"].to_numpy(),
                label=f"{i}",
                zorder=max(veh_inds) - i,
                color="C{}".format(i),
                linewidth=linewidth,
                linestyle=linestyle,
                alpha=alphas[i],
            )
            ax[1, j].plot(
                data["time"].to_numpy(),
                data["ego_speed"].to_numpy(),
                label=f"{i}",
                zorder=max(veh_inds) - i,
                color="C{}".format(i),
                linewidth=linewidth,
                linestyle=linestyle,
                alpha=alphas[i],
            )
            if i > 0:
                ax[2, j].plot(
                    data["time"].to_numpy(),
                    data["dist_err"].to_numpy(),
                    label=f"{i}",
                    zorder=max(veh_inds) - i,
                    color="C{}".format(i),
                    linewidth=linewidth,
                    linestyle=linestyle,
                    alpha=alphas[i],
                )
                ax[3, j].plot(
                    data["time"].to_numpy(),
                    data["speed_err"].to_numpy(),
                    label=f"{i}",
                    zorder=max(veh_inds) - i,
                    color="C{}".format(i),
                    linewidth=linewidth,
                    linestyle=linestyle,
                    alpha=alphas[i],
                )
    ax[0, 0].set_ylabel("position [m]", fontsize=axes_size)
    ax[1, 0].set_ylabel("velocity [m/s]", fontsize=axes_size)
    ax[2, 0].set_ylabel("spacing error [m]", fontsize=axes_size)
    ax[3, 0].set_ylabel("velocity error [m/s]", fontsize=axes_size)
    ax[3, 0].set_xlabel("time [s]", fontsize=axes_size)
    ax[3, 1].set_xlabel("time [s]", fontsize=axes_size)
    for a in ax.flat:
        a.grid()
        a.tick_params(axis="both", labelsize=tick_label_size)
    ax[1, 1].legend(
        bbox_to_anchor=(1.0, 0.0),
        loc="center left",
        fontsize=legend_font_size,
    )
    fig_loc = os.path.join(prefix, "data_analysis/figures/")
    fig_file = os.path.join(fig_loc, "itsc_2024_hardware_results.pdf")
    plt.savefig(fig_file, bbox_inches="tight", pad_inches=0)
    plt.show()

    # calculate RMSE
    dfs = [cth_df, cdh_df]
    labels = ["CTH", "CDH"]
    veh_inds = [i for i in range(1, max(cth_df["veh_ind"]) + 1)]
    for df, label in zip(dfs, labels):
        for i in veh_inds:
            data = df[df["veh_ind"] == i]
            dist_err = data["dist_err"].to_numpy()
            speed_err = data["speed_err"].to_numpy()
            dist_mean = dist_err.mean()
            dist_std = dist_err.std()
            dist_rmse = (sum(dist_err ** 2) / len(dist_err)) ** 0.5
            speed_rmse = (sum(speed_err ** 2) / len(speed_err)) ** 0.5
            print(f"{label} Vehicle {i} Distance RMSE: {dist_rmse:.2f}")
            print(f"{label} Vehicle {i} Speed RMSE: {speed_rmse:.2f}")