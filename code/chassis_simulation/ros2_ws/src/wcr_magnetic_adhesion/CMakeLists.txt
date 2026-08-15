cmake_minimum_required(VERSION 3.8)
project(wcr_magnetic_adhesion)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(wcr_magnetic_adhesion_system SHARED
  src/magnetic_adhesion_system.cpp
)

target_compile_features(wcr_magnetic_adhesion_system PUBLIC cxx_std_17)
target_link_libraries(wcr_magnetic_adhesion_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS wcr_magnetic_adhesion_system
  LIBRARY DESTINATION lib
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/wcr_magnetic_adhesion.dsv.in"
)

ament_package()
