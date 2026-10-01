cmake_minimum_required(VERSION 3.8)
project(robotbase_keyop)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robotbase_keyop_v2 src/robotbase_keyop_v2.cpp)
target_compile_features(robotbase_keyop_v2 PUBLIC cxx_std_17)
ament_target_dependencies(robotbase_keyop_v2 geometry_msgs rclcpp std_msgs)

install(TARGETS robotbase_keyop_v2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
