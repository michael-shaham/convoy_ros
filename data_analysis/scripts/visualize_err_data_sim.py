import matplotlib.pyplot as plt
import os
import pandas as pd

plt.rcParams["font.family"] = "Times New Roman"


if __name__ == "__main__":
    test_name = input("Enter the test name: ")

    save_file = (
        os.path.expanduser("~")
        + f"/cvy_ws/src/convoy_ros/data_analysis/scripts/figures/{test_name}_error.pdf"
    )
    df = pd.read_csv("~/cvy_ws/src/convoy_ros/convoy_data/data/" + test_name + ".csv")
    df_errors = df[["veh_ind", "time", "dist_err", "speed_err"]]
    n_vehicles = 7

    # don't really use these right now but can if needed
    title_size = 26
    subtitle_size = 22
    axes_size = 18
    legend_size = 16
    tick_label_size = 14

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 10))
    fig.suptitle(f"{test_name} error: 7 simulated vehicles", fontsize=title_size)

    for i in range(1, n_vehicles):
        veh_err = df_errors[df_errors["veh_ind"] == i]
        ax[0].plot(
            veh_err["time"].to_numpy(),
            veh_err["dist_err"].to_numpy(),
            label="veh_" + str(i),
        )

        ax[1].plot(
            veh_err["time"].to_numpy(),
            veh_err["speed_err"].to_numpy(),
            label="veh_" + str(i),
        )

    ax[0].set_ylabel("distance error [m]")
    ax[1].set_ylabel("speed error [m]")
    ax[1].set_xlabel("time [s]")
    ax[0].grid()
    ax[1].grid()
    ax[0].legend(bbox_to_anchor=(1.01, 0.5), loc="center left")
    ax[1].legend(bbox_to_anchor=(1.01, 0.5), loc="center left")
    fig.subplots_adjust(0.06, 0.08, 0.89, 0.92, 0.20, 0.11)
    fig.savefig(save_file, bbox_inches="tight")
    plt.show()
