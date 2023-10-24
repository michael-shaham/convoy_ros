# Convoy ROS

This is a ROS package for controlling both simulated and real one-tenth scale 
vehicles that will be used to test and evaluate convoy algorithms.

We will be using ROS2 for this project. For now, we will use the Foxy 
distribution. See the `docs` folder for various resources on vehicle setup, 
running the simulator, network setup, and more.

## TODO

### Hardware

- [ ] put together vehicle 7

### General

- [x] implement data logger
- [x] compare simulation data between linear feedback and PFMPC
- [ ] compare simulation data between neural network controllers and classical 
      controllers
- [x] compare real data between linear feedback and PFMPC

### Head vehicle algorithms

#### Perception

- [ ] improve segmentation of course walls (low priority, works well)

#### Planning

- [ ] improve planner (low priority)

#### Control

- [ ] debug Stanley controller on hardware
- [x] develop double integrator longitudinal controller

### Convoy algorithms

#### Perception

#### Planning

- [ ] improve constrained planner (low priority)

#### Control

- [x] test distributed MPC algorithms on hardware
- [x] learn a neural network controller based on DMPC and/or linear feedback
- [ ] bootstrap RL algorithm to improve neural network controllers
