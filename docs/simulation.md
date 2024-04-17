# F1Tenth simulator installation and setup

## Installation

To launch the f1tenth simulator, first make sure you have followed all of the 
installation instructions at 
[`f1tenth_gym_ros`](https://www.github.com/f1tenth/f1tenth_gym_ros).
Make sure you properly install Docker (including post-installation steps), 
nvidia-docker2, and rocker.

## Simulator setup (without Docker)

These instructions essentially follow the Dockerfile.

Run the following commands:

```bash
sudo apt update
sudo apt upgrade
sudo apt install git nano vim python3-pip libeigen3-dev tmux
pip3 install transforms3d scikit-learn cvxpy xacro
cd ~ && git clone git@github.com:RIVeR-Lab/f1tenth_gym.git
cd f1tenth_gym && pip3 install -e .
```

Now, if you haven't already, create a workspace called `cvy_ws` and clone this 
package into it:

```bash
mkdir -p ~/cvy_ws/src
cd ~/cvy_ws/src
git clone git@github.com:RIVeR-Lab/convoy_ros.git
cd convoy_ros && git submodule init && git submodule update 
```

Now in a terminal, run:

```bash
cd ~/cvy_ws/
source /opt/ros/foxy/setup.bash
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

Finally, to start the simulator, run:

```bash
ros2 launch convoy_bringup sim_veh_bringup.launch.py
```

## Simulator setup (Docker)

To launch the simulator, navigate to the `convoy` package directory and run

### With an NVIDIA GPU

```bash
docker build -t convoy -f Dockerfile .
rocker --nvidia --x11 --volume .:/root/sim_ws/src/convoy_ros/ --volume /home/$(echo $USER)/.bashrc:/root/.bashrc --volume /home/$(echo $USER)/.vimrc_simple:/root/.vimrc --volume /home/$(echo $USER)/.tmux.conf:/root/.tmux.conf -- convoy
```

Note that the `dot-files` volume is optional. The above uses Michael's dot 
files in the container for better readability/code-editing in vim/tmux.

### Without an NVIDIA GPU

In the first terminal, run

```bash
docker build -t convoy -f Dockerfile .
docker-compose-v1 up
```

Michael's Macbook Air requires specifying the v1. I am not sure if this is 
specific to Macs (or even Apple Silicon Macs) or if it is a new thing. The 
docker-compose command throws an error.

You will now be able to navigate to 
[http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) and hit 
`Connect`. 
In another terminal, run

```bash
docker exec -it convoy_sim_1 /bin/bash
```

This will place you into a terminal environment within the container where you 
can launch the simulator, run nodes, etc.

Note that the `docker-compose.yml` file specifies a mounted volume specific to 
Michael's Macbook Air (Michael's dot files). Either remove this line or change 
it accordingly (don't push it if you change it).

## Running and visualizing the simulator

### With an NVIDIA GPU

In the Docker container (in the same terminal you ran the `rocker` command in), 
run

```bash
ros2 launch convoy_bringup sim_veh_bringup.launch.py
```

and then you should see RViz open.

### Without an NVIDIA GPU

In the Docker container (the terminal you ran the `exec` command in), run

```bash
ros2 launch convoy_bringup sim_veh_bringup.launch.py
```

This does not work on Michael's Macbook Air, though it works on his desktop 
with a GPU. RViz loads but never shows a vehicle or the map, not sure why.

