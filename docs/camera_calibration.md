# Camera calibration

These instructions mostly follow the 
[camera calibration tutorials](https://navigation.ros.org/tutorials/docs/camera_calibration.html) 
from the Nav2 documentation. Currently, the ROS 2 camera calibration package 
binaries don't work, so the package needs to be built from source:

```bash
cd ~/cvy_ws/src/
git clone -b ros2 https://github.com/ros-perception/image_pipeline.git
cd ~/cvy_ws/
rosdep install -i --from-path src --rosdistro foxy -y
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build --symlink-install
```

Eventually, we should not have to do this (once they update the binaries).

Let's assume we are calibrating the camera for vehicle $x$. To bring up the 
camera, first source everything and then run (be sure to replace $x$ with the 
vehicle number):

```bash
ros2 launch convoy_bringup arducam.launch.py veh_ns:=veh_x show_cam:=true rectify_cam:=false
```

If everything works properly, you should see the `/veh_x/image_raw` topic on 
your screen. Now we run the `cameracalibrator` node. If you are using the 
checkerboard Michael printed that is kept by the convoy desk, the size to use 
is 5x7 and the square sizes are 0.03 meters (30 mm). We will also call the 
camera `arducam` in the configuration files. To start the node, run in a 
second terminal (be sure to replace $x$ with the vehicle number):

```bash
ros2 run camera_calibration cameracalibrator --size 5x7 --square 0.03 --camera_name arducam --ros-args --remap image:=/veh_x/image_raw -p camera:=/veh_x
```

The camera calibration application should pop up. After you are finished 
calibrating and the `CALIBRATE` button turns blue, click the button. Once 
`SAVE` turns blue, click it, and then you should see in the calibration node's 
terminal that the calibration file was written to `/tmp/calibrationdata.tar.gz`.
To extract the contents and move the file to the correct location in the 
`convoy_bringup` package, run the following (making sure to replace $x$ with the 
vehicle number):

```bash
cd /tmp/ && mkdir calibrationdata
tar -zxvf calibrationdata.tar.gz -C calibrationdata/
cp /tmp/calibrationdata/ost.yaml ~/cvy_ws/src/convoy/convoy_bringup/config/veh_x/arducam_info.yaml
```

Now to test that it worked, run the following (replace $x$ with vehicle number):

```bash
ros2 launch convoy_bringup arducam.launch.py veh_ns:=veh_x show_cam:=true
```

There should now be two video streams that appear on the screen. If you place 
the checkerboard close to the camera, you should see that in one stream the 
lines in the checkerboard appear curved (the raw image) and in the other stream 
the lines appear straight (the rectified image).
