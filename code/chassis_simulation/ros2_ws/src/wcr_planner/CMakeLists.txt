cmake_minimum_required(VERSION 3.8)
project(wcr_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wcr_planning_msgs REQUIRED)

add_executable(online_trajectory_planner src/online_trajectory_planner.cpp)
ament_target_dependencies(online_trajectory_planner
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  wcr_planning_msgs
)

install(TARGETS online_trajectory_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
