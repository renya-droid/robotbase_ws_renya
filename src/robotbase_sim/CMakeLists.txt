cmake_minimum_required(VERSION 3.8)
project(robotbase_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps models rviz worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    scripts/generate_slope_world.py
    scripts/idle_twist_publisher.py
    scripts/velodyne_clearing_scenario.py
    scripts/velodyne_coverage_viz.py
    scripts/velodyne_slope_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
