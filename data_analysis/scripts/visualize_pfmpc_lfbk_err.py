import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd

plt.rcParams["font.family"] = "Times New Roman"


if __name__ == "__main__":
    data_loc = os.path.expanduser("~") + "/cvy_ws/src/convoy_ros/convoy_data/data/"
    data_files = os.listdir(data_loc)
    data_files.sort()
    lfbk_files = [file for file in data_files if "lfbk_sim_1.5-3" in file]
    pfmpc_files = [file for file in data_files if "pfmpc_sim_1.5-3" in file]

    save_file = (
        os.path.expanduser("~")
        + f"/cvy_ws/src/convoy_ros/data_analysis/figures/sim_performance_compare.pdf"
    )

    lfbk_df = pd.DataFrame()
    pfmpc_df = pd.DataFrame()
    for file in lfbk_files:
        lfbk_df = pd.concat([lfbk_df, pd.read_csv(data_loc + file)])
    for file in pfmpc_files:
        pfmpc_df = pd.concat([pfmpc_df, pd.read_csv(data_loc + file)])

    pfmpc_df_errors = pfmpc_df[
        ["veh_ind", "time", "dist_err", "speed_err", "ego_speed", "opt_input"]
    ]
    lfbk_df_errors = lfbk_df[
        ["veh_ind", "time", "dist_err", "speed_err", "ego_speed", "opt_input"]
    ]
    n_vehicles = 7

    stop_time = 42.0
    lfbk_df_errors = lfbk_df_errors[lfbk_df_errors["time"] <= stop_time]
    pfmpc_df_errors = pfmpc_df_errors[pfmpc_df_errors["time"] <= stop_time]

    # don't really use these right now but can if needed
    title_size = 15
    subtitle_size = 12
    axes_size = 10
    legend_size = 10
    tick_label_size = 10
    linewidth = 1.0
    linestyle = "solid"
    alphas = [0.9 for _ in range(n_vehicles)]

    fig, ax = plt.subplots(2, 2, sharex="col", sharey="row", figsize=(4.65, 3))
    fig.suptitle(
        "Platoon performance comparison: 1.5-3 m/s alternating", fontsize=title_size
    )
    ax[0, 0].set_title("Distributed MPC", fontsize=subtitle_size)
    ax[0, 1].set_title("Linear feedback", fontsize=subtitle_size)

    # ax[0, 1].text(1.03, 0.5, "e < 0 : too far", va='center',
    #               transform=ax[0, 1].transAxes)
    # ax[1, 1].text(1.03, 0.5, "e < 0 : too slow", va='center',
    #               transform=ax[1, 1].transAxes)

    veh_inds = [
        i for i in range(1, n_vehicles) if i in pfmpc_df_errors["veh_ind"].to_numpy()
    ]

    pfmpc_dist_err_rms = {}
    pfmpc_speed_err_rms = {}
    lfbk_dist_err_rms = {}
    lfbk_speed_err_rms = {}
    for k, i in enumerate(veh_inds):
        pfmpc_veh_err = pfmpc_df_errors[pfmpc_df_errors["veh_ind"] == i]
        lfbk_veh_err = lfbk_df_errors[lfbk_df_errors["veh_ind"] == i]

        pfmpc_time = pfmpc_veh_err["time"].to_numpy()
        pfmpc_dist_err = pfmpc_veh_err["dist_err"].to_numpy()
        pfmpc_speed_err = pfmpc_veh_err["speed_err"].to_numpy()

        lfbk_time = lfbk_veh_err["time"].to_numpy()
        lfbk_dist_err = lfbk_veh_err["dist_err"].to_numpy()
        lfbk_speed_err = lfbk_veh_err["speed_err"].to_numpy()

        pfmpc_dist_err_rms[i] = np.sqrt(
            np.sum(np.square(pfmpc_dist_err)) / len(pfmpc_dist_err)
        )
        pfmpc_speed_err_rms[i] = np.sqrt(
            np.sum(np.square(pfmpc_speed_err)) / len(pfmpc_speed_err)
        )

        lfbk_dist_err_rms[i] = np.sqrt(
            np.sum(np.square(lfbk_dist_err)) / len(lfbk_dist_err)
        )
        lfbk_speed_err_rms[i] = np.sqrt(
            np.sum(np.square(lfbk_speed_err)) / len(lfbk_speed_err)
        )

        ax[0, 0].plot(
            pfmpc_veh_err["time"].to_numpy(),
            pfmpc_veh_err["dist_err"].to_numpy(),
            label=f"{k+1}",
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[k],
        )
        ax[0, 1].plot(
            lfbk_veh_err["time"].to_numpy(),
            lfbk_veh_err["dist_err"].to_numpy(),
            label=f"{k+1}",
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[k],
        )

        # ax[1,0].plot(pfmpc_veh_err['time'].to_numpy(),
        #              pfmpc_veh_err['speed_err'].to_numpy(),
        #              label=f"{k+1}",
        #              linewidth=linewidth, linestyle=linestyle, alpha=alphas[k])
        # ax[1,1].plot(lfbk_veh_err['time'].to_numpy(),
        #              lfbk_veh_err['speed_err'].to_numpy(),
        #              label=f"{k+1}"
        #              linewidth=linewidth, linestyle=linestyle, alpha=alphas[k])

        ax[1, 0].plot(
            pfmpc_veh_err["time"].to_numpy(),
            pfmpc_veh_err["ego_speed"].to_numpy(),
            label=f"{k+1}",
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[k],
        )
        ax[1, 1].plot(
            lfbk_veh_err["time"].to_numpy(),
            lfbk_veh_err["ego_speed"].to_numpy(),
            label=f"{k+1}",
            linewidth=linewidth,
            linestyle=linestyle,
            alpha=alphas[k],
        )

        # ax[3,0].plot(pfmpc_veh_err['time'].to_numpy(),
        #              pfmpc_veh_err['opt_input'].to_numpy(),
        #              label=f"{k+1}",
        #              linewidth=linewidth, linestyle=linestyle, alpha=alphas[k])
        # ax[3,1].plot(lfbk_veh_err['time'].to_numpy(),
        #              lfbk_veh_err['opt_input'].to_numpy(),
        #              label=f"{k+1}",
        #              linewidth=linewidth, linestyle=linestyle, alpha=alphas[k])

    ax[0, 0].set_ylabel("distance error [m]", fontsize=axes_size)
    # ax[1,0].set_ylabel('speed error [m/s]', fontsize=axes_size)
    ax[1, 0].set_ylabel("speed [m/s]", fontsize=axes_size)
    # ax[3,0].set_ylabel('input[m/s]', fontsize=axes_size)
    ax[1, 0].set_xlabel("time [s]", fontsize=axes_size)
    ax[1, 1].set_xlabel("time [s]", fontsize=axes_size)

    for a in ax.flat:
        a.grid()
        a.tick_params(axis="both", labelsize=tick_label_size)
    for a in ax[1, :]:
        a.set_yticks([0.0, 1.5, 3.0])

    ax[1, 1].legend(bbox_to_anchor=(1.01, 1.0), loc="center left", fontsize=legend_size)

    fig.subplots_adjust(0.13, 0.16, 0.84, 0.82, 0.07, 0.09)
    plt.show()

    for i in veh_inds:
        print(f"vehicle {i}:")
        print(f"\tpfmpc dist rms: {pfmpc_dist_err_rms[i]}")
        print(f"\tpfmpc speed rms: {pfmpc_speed_err_rms[i]}")
        print(f"\tlfbk dist rms: {lfbk_dist_err_rms[i]}")
        print(f"\tlfbk speed rms: {lfbk_speed_err_rms[i]}\n")

    fig.savefig(save_file, bbox_inches="tight")
