# Vehicle bringup

## SSH into the vehicle

Each vehicle is assigned a number $\{0,\ldots,N-1\}$ where $N$ is the number of 
vehicles we have. Let $x$ be the vehicle number. Then the vehicle will have IP 
address `192.168.1.10x` (assuming we don't have more than 10 vehicles). To ssh 
into the vehicle:

```bash
ssh river@192.168.1.10x
```

where $x$ is the vehicle number. The password will be `convoyTF!40`.

## Bringing up the vehicle

It is **highly** recommended to use `tmux` and `vim` (but more importantly 
tmux). Ask Michael about his tmux configuration files (which are on the
vehicles) if you want to learn how to use his shortcuts.

After sshing into the vehicle, run the following in each terminal:

```bash
source-vehicle x
```

This will source the foxy distribution, the convoy workspace (`~/cvy_ws/`), and 
set the `ROS_DOMAIN_ID` parameter to $x$ (see Michael's `.bashrc` file, 
which Michael sets in each vehicle, to find out exactly what the function does).

Then launch the vehicle:

```bash
ros2 launch convoy_bringup real_veh_bringup.launch.py veh_ns:=veh_x lidar:=hokuyo use_cam:=false
```

This assumes we are using the Hokuyo lidar without a RealSense. 
If you are using the RPLiDAR S2, use `rplidar` in place of `hokuyo`. If you 
want to get more information about the ROS launch arguments, use the following 
command:

```bash
ros2 launch convoy_bringup veh_bringup.launch.py --show-args
```
