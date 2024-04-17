cmake_minimum_required(VERSION 3.5)
project(convoy_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# make launch, config, etc. files findable

install(
  DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}/
)

# python stuff

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/drive_controller.py
  scripts/full_control/bicycle_mpc.py
  scripts/lateral_control/pure_pursuit.py
  scripts/lateral_control/stanley.py
  scripts/longitudinal_control/constant_input.py
  scripts/longitudinal_control/constant_current.py
  scripts/longitudinal_control/dmpc.py
  scripts/longitudinal_control/double_int_mpc.py
  scripts/longitudinal_control/linear_feedback.py
  scripts/longitudinal_control/linear_mpc.py
  scripts/longitudinal_control/l1_pfmpc.py
  scripts/longitudinal_control/quad_pfmpc.py
  scripts/longitudinal_control/track_vl.py
  scripts/longitudinal_control/virtual_leader.py
  scripts/longitudinal_control/nn_feedback.py
  DESTINATION lib/${PROJECT_NAME}
)

# install the package

ament_package()
