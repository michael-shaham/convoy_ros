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
    range_data = []
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
            neighbor_data.append(neighbor)

            ranges = np.loadtxt(data_path + f"veh_{i}_range_data.csv")
            ranges[0, :] -= ranges[0, 0]
            ranges = ranges[:, ranges[1, :] < 10]
            ranges = ranges[:, ranges[1, :] > 0]
            range_data.append(ranges)

    return drive_data, neighbor_data, range_data, speed_data


if __name__ == "__main__":
    test_name = "sine_attack_3"
    desired_distance = 1.0
    attack_distance = 0.4
    n_vehicles = 3
    drive_data, neighbor_data, range_data, speed_data = load_data(test_name, n_vehicles)

    title_size = 16
    subtitle_size = 14
    axes_size = 10
    legend_size = 12
    tick_label_size = 9

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 5))
    fig.suptitle(
        "Convoy dynamics - 3 vehicles with sensor attack on vehicle 1",
        fontsize=title_size,
    )

    # visualize speed data
    zero_start_time = neighbor_data[0][0, (neighbor_data[0][2] != 0).argmax()]
    one_start_time = neighbor_data[1][0, (neighbor_data[1][2] != 0).argmax()]
    two_start_time = one_start_time
    start_times = [zero_start_time, one_start_time, two_start_time]
    for i in range(n_vehicles - 1):
        ax[0].plot(
            neighbor_data[i][0] - start_times[i],
            neighbor_data[i][2],
            label=f"vehicle {i}",
        )
    ax[0].plot(speed_data[2][0] - start_times[2], speed_data[2][1], label="vehicle 2")
    ax[0].set_ylabel("speed [m/s]", fontsize=axes_size)
    ax[0].set_title("Speed vs time", fontsize=subtitle_size)
    ax[0].tick_params(labelsize=tick_label_size)
    ax[0].grid()
    ax[0].legend(loc="center left", bbox_to_anchor=(1, 0.5), prop={"size": legend_size})

    # visualize distance data
    # for i in range(n_vehicles - 1):
    #     ax[1].plot(neighbor_data[i][0] - start_times[i], neighbor_data[i][1],
    #                label=rf"vehicle ${i+1}$ to ${i}$")
    # ax[1].plot(neighbor_data[1][0] - start_times[2],
    #            desired_distance * np.ones_like(neighbor_data[1][0]),
    #            label=f"desired distance: {desired_distance}")
    # ax[1].set_ylabel("distance [m]", fontsize=axes_size)
    # ax[1].set_title("Perceived distance vs time", fontsize=subtitle_size)
    # ax[1].tick_params(labelsize=tick_label_size)
    # ax[1].grid()
    # ax[1].legend(loc='center left', bbox_to_anchor=(1, 0.5),
    #              prop={'size': legend_size})

    # visualize distance data
    for i in range(n_vehicles - 1):
        if i == 0:
            ax[1].plot(
                neighbor_data[i][0] - start_times[i],
                neighbor_data[i][1] - attack_distance,
                label=f"vehicle {i+1} to {i}",
            )
        else:
            ax[1].plot(
                neighbor_data[i][0] - start_times[i],
                neighbor_data[i][1],
                label=f"vehicle {i+1} to {i}",
            )
    ax[1].plot(
        neighbor_data[1][0] - start_times[1],
        desired_distance * np.ones_like(neighbor_data[i][0]),
        label=f"desired distance: {desired_distance}",
    )
    ax[1].set_xlabel("time [s]", fontsize=axes_size)
    ax[1].set_ylabel("distance [m]", fontsize=axes_size)
    ax[1].set_title("True distance vs time", fontsize=subtitle_size)
    ax[1].tick_params(labelsize=tick_label_size)
    ax[1].grid()
    ax[1].legend(loc="center left", bbox_to_anchor=(1, 0.5), prop={"size": legend_size})

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
    plt.xlim(-2, 66)
    plt.subplots_adjust(
        left=0.05, bottom=0.10, right=0.79, top=0.87, wspace=0.20, hspace=0.30
    )
    plt.subplot_tool()
    fig.savefig(save_file, bbox_inches="tight", dpi=400)
    plt.show()

    # Get RMS error for distance
    # neighbor_data[0][1, :] -= attack_distance
    distance_errors = []
    distance_rms_errors = []
    for i in range(n_vehicles - 1):
        neighbor_data[i][0] -= start_times[i]
        neighbor_data[i] = neighbor_data[i][:, neighbor_data[i][0, :] >= 0.0]
        neighbor_data[i] = neighbor_data[i][:, neighbor_data[i][0, :] <= 63.5]
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
        speed_data[i][0] -= start_times[i]
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
