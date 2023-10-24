cmake_minimum_required(VERSION 3.5)
project(convoy_planning)

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

# make launch, config, etc. folders findable
install(
  DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables
install(PROGRAMS
  scripts/poly_planner_head.py
  scripts/poly_planner_trail.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
