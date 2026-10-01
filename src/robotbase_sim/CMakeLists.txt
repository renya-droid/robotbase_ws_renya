cmake_minimum_required(VERSION 3.8)
project(robotbase_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps models worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS scripts/idle_twist_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
