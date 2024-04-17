import matplotlib.pyplot as plt
import numpy as np
import os

plt.rcParams["font.family"] = "Times New Roman"


def load_data(test_name: str, n_vehicles: int):
    """
    drive data: shape 4xN, rows are (time, speed, accel, steer)
    neighbor data: shape 3xN, rows are (time, distance, speed)
    speed data: Shape 2xN, rows are (time, speed)
    range data: Shape 2xN, rows are (time, range)
    """
    data_path = (
        os.path.join(
            os.path.expanduser("~"),
            "cvy_ws",
            "src",
            "convoy_ros",
            "data_analysis",
            "data",
            test_name,
        )
        + "/"
    )
    drive_data = []
    neighbor_data = []
    speed_data = []
    for i in range(n_vehicles):
        drive = np.loadtxt(data_path + f"veh_{i}_drive_data.csv")
        drive[0, :] -= drive[0, 0]
        drive_data.append(drive)

        speed = np.loadtxt(data_path + f"veh_{i}_speed_data.csv")
        speed[0, :] -= speed[0, 0]

        # if i > 0:
        #     speed = speed[:, speed[0, :] < 0]
        # speed[0, :] -= speed[0, 0]
        speed_data.append(speed)
        if i > 0:
            neighbor = np.loadtxt(data_path + f"veh_{i}_neighbor_data.csv")
            neighbor[0, :] -= neighbor[0, 0]
            # neighbor = neighbor[:, neighbor[0, :] > 100]
            # neighbor[0, :] -= neighbor[0, 0]
            neighbor = neighbor[:, neighbor[1, :] < 10]
            neighbor = neighbor[:, neighbor[1, :] > 0]
            ma_window = 20
            for i in range(len(neighbor[0, :])):
                if i >= ma_window:
                    neighbor[1, i] = (
                        neighbor[1, i - ma_window + 1 : i + 1].sum() / ma_window
                    )
            neighbor_data.append(neighbor)

    return drive_data, neighbor_data, speed_data


if __name__ == "__main__":
    test_name = "sim/sim_attack_1"
    desired_distance = 1.0
    n_vehicles = 7
    attack_distance = 0.4
    drive_data, neighbor_data, speed_data = load_data(test_name, n_vehicles)

    title_size = 26
    subtitle_size = 22
    axes_size = 18
    legend_size = 16
    tick_label_size = 14

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 10))
    fig.suptitle("Convoy dynamics - 7 simulated vehicles", fontsize=title_size)

    # visualize speed data
    start_time = speed_data[0][0, (speed_data[0][1] != 0).argmax() - 1]
    print(len(speed_data))
    for i in range(n_vehicles):
        ax[0].plot(
            speed_data[i][0] - start_time, speed_data[i][1], label=f"vehicle {i}"
        )
    ax[0].set_ylabel("speed [m/s]", fontsize=axes_size)
    ax[0].set_title("Speed vs time", fontsize=subtitle_size)
    ax[0].tick_params(labelsize=tick_label_size)
    ax[0].grid()
    ax[0].legend(loc="center left", bbox_to_anchor=(1, 0.5), prop={"size": legend_size})

    # visualize distance data
    for i in range(n_vehicles - 1):
        ax[1].plot(
            neighbor_data[i][0], neighbor_data[i][1], label=f"vehicle {i+1} to {i}"
        )
    ax[1].plot(
        neighbor_data[1][0],
        desired_distance * np.ones_like(neighbor_data[1][0]),
        label=f"desired distance: {desired_distance}",
    )
    ax[1].set_ylabel("distance [m]", fontsize=axes_size)
    ax[1].set_title("Perceived distance vs time", fontsize=subtitle_size)
    ax[1].tick_params(labelsize=tick_label_size)
    ax[1].grid()
    ax[1].legend(loc="center left", bbox_to_anchor=(1, 0.5), prop={"size": legend_size})

    # visualize distance data
    for i in range(n_vehicles - 1):
        if i == 0:
            ax[2].plot(
                neighbor_data[i][0],
                neighbor_data[i][1] - attack_distance,
                label=f"vehicle {i+1} to {i}",
            )
        else:
            ax[2].plot(
                neighbor_data[i][0], neighbor_data[i][1], label=f"vehicle {i+1} to {i}"
            )
    ax[2].plot(
        neighbor_data[1][0],
        desired_distance * np.ones_like(neighbor_data[i][0]),
        label=f"desired distance: {desired_distance}",
    )
    ax[2].set_xlabel("time [s]", fontsize=axes_size)
    ax[2].set_ylabel("distance[m]", fontsize=axes_size)
    ax[2].set_title("True distance vs time", fontsize=subtitle_size)
    ax[2].tick_params(labelsize=tick_label_size)
    ax[2].grid()
    ax[2].legend(loc="center left", bbox_to_anchor=(1, 0.5), prop={"size": legend_size})

    plt.xlim(-2, 35)
    save_file = os.path.join(
        os.path.expanduser("~"),
        "cvy_ws",
        "src",
        "convoy_ros",
        "data_analysis",
        "scripts",
        "figures",
        test_name + ".pdf",
    )
    plt.subplots_adjust(
        left=0.07, bottom=0.07, right=0.72, top=0.90, wspace=0.20, hspace=0.24
    )
    fig.savefig(save_file, bbox_inches="tight", dpi=400)
    plt.subplot_tool()
    plt.show()

    # Get RMS error for distance
    # neighbor_data[0][1, :] -= attack_distance
    distance_errors = []
    distance_rms_errors = []
    for i in range(n_vehicles - 1):
        neighbor_data[i][0] -= start_time
        neighbor_data[i] = neighbor_data[i][:, neighbor_data[i][0, :] >= 0.0]
        # neighbor_data[i] = neighbor_data[i][:, neighbor_data[i][0, :] <= 63.5]
        distance_errors.append(neighbor_data[i][1, :] - desired_distance)
        distance_rms_errors.append(np.sqrt((distance_errors[i] ** 2).mean()))
        print(f"Vehicle {i+1} distance RMS error: {distance_rms_errors[i]} m")

    plt.figure()
    for i in range(n_vehicles - 1):
        plt.plot(
            range(len(distance_errors[i])), distance_errors[i], label=f"vehicle {i + 1}"
        )
    plt.title("Distance error vs time")
    plt.ylabel("error [m]")
    plt.grid()
    plt.legend()
    # plt.show()

    # Get RMS error for speed
    speed_errors = []
    speed_rms_errors = []
    for i in range(n_vehicles):
        speed_data[i][0] -= start_time
        speed_data[i] = speed_data[i][:, speed_data[i][0, :] >= 0.0]
        speed_data[i] = speed_data[i][:, speed_data[i][0, :] <= 63.5]

    for i in range(n_vehicles - 1):
        speed_errors.append(speed_data[i][1, :] - speed_data[i + 1][1, :])
        speed_rms_errors.append(np.sqrt((speed_errors[i] ** 2).mean()))
        print(f"Vehicle {i+1} speed RMS error: {speed_rms_errors[i]} m/s")

    plt.figure()
    for i in range(n_vehicles - 1):
        plt.plot(range(len(speed_errors[i])), speed_errors[i], label=f"vehicle {i + 1}")
    plt.title("Speed error vs time")
    plt.ylabel("error [m/s]")
    plt.grid()
    plt.legend()
    # plt.show()
