cmake_minimum_required(VERSION 3.8)
project(arm_test)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/test_sequence.py
  scripts/calib_collect.py
  scripts/ik_node.py
  scripts/hand_eye_calibrate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
