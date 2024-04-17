cmake_minimum_required(VERSION 3.5)
project(convoy_interfaces)

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# declare messages and services to generate
set(msg_files
  "msg/ConvoyControlData.msg"
  "msg/CourseData.msg"
  "msg/DesiredDistance.msg"
  "msg/LateralControl.msg"
  "msg/LongitudinalControl.msg"
  "msg/LongitudinalState.msg"
  "msg/LongitudinalStateTrajectory.msg"
  "msg/NeighborInfo.msg"
  "msg/SafeSpeed.msg"
  "msg/Trajectory.msg"
)
set(srv_files
)

# generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_package()
