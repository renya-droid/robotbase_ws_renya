cmake_minimum_required(VERSION 3.8)
project(robotbase_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS scripts/hokuyo_no_return_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
