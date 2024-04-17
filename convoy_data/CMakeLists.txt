cmake_minimum_required(VERSION 3.5)
project(convoy_data)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# include launch, config, etc. folders
install(
  DIRECTORY
  launch
  plotjuggler
  DESTINATION share/${PROJECT_NAME}/
)

# python stuff

install(PROGRAMS
  scripts/bag_process.py
  scripts/bag_process_sim.py
  scripts/log_data.py
  DESTINATION lib/${PROJECT_NAME}
)

# install the package

ament_package()
