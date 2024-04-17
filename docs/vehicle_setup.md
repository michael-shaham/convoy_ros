# Vehicle setup

## Vehicle naming (when installing Ubuntu)

Each vehicle is assigned a number, starting at 0. When installing Ubuntu 20 on 
the computer, use the below names when filling out the `Who are you?` section. 
Let's assume we are setting up vehicle 4.

| Section | Information |
| ------- | ----------- |
| Your name: | RIVeR-Lab |
| Your computer's name: | convoy-4 |
| Pick a username: | river |
| Choose a password: | convoyTF!40 |

## Workspace/software setup

Make sure you have `git`, `tmux`, `vim`, and other essential software installed.
If desired, follow the steps in the `misc.md` to map `caps` to `escape` or to 
both `escape` and `control` (using keyd). You can also follow `dot_files.md` to 
set up various files for the vehicle to make life easier.

If you want to also pull submodules when performing `git pull`, run the 
following:

```bash
git config --global submodule.recurse true
```

Also, make sure to 
[set up an ssh key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) 
for GitHub on the vehicle.

Now install ROS 2. Afterwards, be sure to install colcon, rosdep, 
openssh-server (so we can ssh into the vehicle), and OpenCV, and then 
initialize rosdep using:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep openssh-server
sudo apt install python3-pip
pip install opencv-python numpy scikit-learn pypozyx
pip3 install torch --index-url https://download.pytorch.org/whl/cpu
sudo rosdep init
rosdep update
```

Also, add your user to some groups:

```bash
sudo adduser $USER dialout
sudo adduser $USER video
```

To set up a workspace on the vehicle with the appropriate packages:

```bash
git config --global submodule.recurse true
mkdir -p ~/cvy_ws/src/ && cd ~/cvy_ws/src/
git clone git@github.com:RIVeR-Lab/convoy_ros.git
git clone git@github.com:RIVeR-Lab/pozyx_ros.git
git clone git@github.com:RIVeR-Lab/f1tenth_system.git
git clone https://github.com/Slamtec/sllidar_ros2.git  # rplidar
cd ~/cvy_ws/src/convoy_ros/
git submodule update --init --remote
cd ~/cvy_ws/src/f1tenth_system/
git submodule update --init --remote
cd ~/cvy_ws/
rosdep install -i --from-path src --rosdistro foxy -y
source-foxy  # only works if you set up the dot files
colcon build --symlink-install
```

## Bringing up the vehicle(s) on hardware

See the `vehicle_bringup.md` or `testing.md` for information on running.

## Vehicle/device setup notes

Do the following to set up the logitech teleop rules (don't follow the 
directions on the F1Tenth build site):
```bash
sudo vim /etc/udev/rules.d/99-joypad-f710.rules
```
and then paste in the following:
```
KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c21f", SYMLINK+="input/joypad-f710"
```

Do the following to set up the vesc rules (don't follow the directions on the 
F1Tenth build site):
```bash
sudo vim /etc/udev/rules.d/99-vesc.rules
```
and then paste in the following:
```
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{manufacturer}=="STMicroelectronics", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"
```

Do the following to set up the Pozyx device:
```bash
sudo vim /etc/udev/rules.d/99-pozyx.rules
```
and then paste in the following:
```
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{manufacturer}=="Pozyx Labs", SYMLINK+="sensors/pozyx"
```

Do the following to set up the RPLiDAR S2:
```bash
sudo vim /etc/udev/rules.d/99-rplidar-s2.rules
```
and then paste in the following (note that it may actually be `ttyUSB[0-9]*`):
```
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout", SYMLINK+="sensors/rplidar-s2"
```

To put the changes into effect, run

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

and then reboot.

See above sections on bringing up the vehicle to test the lidar and other 
components. If the vehicle is connected to a monitor via HDMI, use the 
`use_rviz:=true` launch argument to visualize the output.

If you run into the issue of not being able to see the LiDAR output, make 
sure the user is in the dialout group, i.e.,

```bash
sudo adduser $USER dialout
```

and then log out and back in for the change to take effect. 

## Some useful USB commands

`lsusb` allows you to see connected USB devices. To figure out where a device 
is connected, run `lsusb` without the device plugged in, and then plug in the 
device and run again.

`usb-devices` lists all of the USB devices with more information including the 
vendor ID and product ID which are needed for creating symbolic links to the 
device.

`dmesg | less` and then hitting `/` and searching for keywords (like the 
product ID or vendor ID) can also be useful.

`sudo udevadm info --name=<device_name> --attribute-walk` where `<device_name>` 
is the location of your device on the system (e.g., `/dev/input/js0`). You can 
also append `| less` to the end of this command to search for the `idVendor` or 
idProduct`.

## Connect Bluetooth devices via the command line

Note: this does not always seem to work, but it has worked in the past to 
connect to a PS4 DS4.

Find the computer's Bluetooth device using the command

```bash
hcitool dev
```

This will return a Bluetooth device name and a MAC address. 
To find Bluetooth devices in range, run

```bash
hcitool -i <dev_name> scan
```

where `dev_name` is likely hci0. The Playstation 4 Dualshock 4 controller will 
be called `Wireless Controller`. Note the MAC address for this device, and 
then trust and connect to the device using the following commands:

```bash
bluetoothctl  # enters you into some terminal environment
trust <MAC_address>
connect <MAC_address>
```
