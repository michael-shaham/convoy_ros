cmake_minimum_required(VERSION 3.5)
project(convoy_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
  DESTINATION share/${PROJECT_NAME}/
)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables
install(PROGRAMS
  scripts/preprocess_lidar.py
  scripts/course_walls_seg.py
  scripts/neighbor_localization_sim.py
  scripts/neighbor_localization_real.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
