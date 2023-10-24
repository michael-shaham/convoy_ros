# Testing

Note that you can use the `--show-args` command at the end of the launch 
command to figure out what options are available for the launch arguments.

## Simulation

### Convoy testing

First make sure you have the gym bridge config file set to only run one agent. 
In two separate terminals, run (after running `source-convoy`)

```bash
ros2 launch convoy_bringup sim_veh_bringup.launch.py
ros2 launch convoy_bringup sim_convoy.launch.py head_long_controller:=<hlong> head_lat_controller:=<hlat> trail_long_controller:=<tlong> trail_lat_controller:=<tlat>
```

## Real

### Head vehicle testing

Terminal one:
```bash
source-convoy
ros2 launch convoy_bringup real_veh_bringup.launch.py veh_ns:=veh_<x> lidar:=<lidar>
```

Terminal two:
```bash
source-convoy
ros2 launch convoy_control head_algorithms.launch.py veh_ns:=veh_<x> long_controller:=<long> lat_controller:=<lat>
```

### Trail vehicle testing

Terminal one:
```bash
source-convoy
ros2 launch convoy_bringup real_veh_bringup.launch.py veh_ns:=veh_<x> lidar:=<lidar>
```

Terminal two:
```bash
source-convoy
ros2 launch convoy_control trail_algorithms.launch.py veh_ns:=veh_<x> neighbor_ind:=<x-1> long_controller:=<long> lat_controller:=<lat>
```
