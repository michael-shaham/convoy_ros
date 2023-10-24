cmake_minimum_required(VERSION 3.5)
project(convoy_learning)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)



install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# python stuff

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/convoy_learn.py
  scripts/crash_detect.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
