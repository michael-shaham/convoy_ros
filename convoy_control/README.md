# Convoy Control

## Simulation demo instructions

For pure pursuit (**note:** only works in simulation for now):

```bash
ros2 launch convoy_bringup sim_veh_bringup.launch.py
ros2 launch convoy_control pure_pursuit.launch.py
```

## Nodes

### pure_pursuit_leader

Obtains waypoints from an external service that loads waypoints from a data 
file for a particular map. Then uses the pure pursuit algorithm to track those 
waypoinst.

### pure_pursuit_follower

Obtains waypoints from an external node that subscribes to the position of the 
leader in simulation. Then uses the waypoints to follower the leader vehicle.
