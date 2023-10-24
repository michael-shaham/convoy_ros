# Running SLAM

```bash
ros2 launch convoy_bringup real_veh_bringup.launch.py veh_ns:=veh_0 lidar:=hokuyo use_cam:=false is_trail:=false
ros2 launch convoy_nav2 slam.launch.py veh_ns:=veh_0
```

Drive around a bit and then to save the map, in a new terminal change to the 
`convoy_bringup/maps/` folder and run

```bash
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p save_map_timeout:=10000
```

# Running localization

```bash
ros2 launch convoy_nav2 amcl.launch.py veh_ns:=veh_<x>
```

where $x$ is the vehicle index.
