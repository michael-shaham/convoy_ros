cmake_minimum_required(VERSION 3.5)
project(convoy_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# install python executables
install(PROGRAMS
  scripts/gym_bridge_multi.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
