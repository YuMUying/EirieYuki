cmake_minimum_required(VERSION 3.8)
project(wcr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env/prepare_envs.sh.in") 

install(
  DIRECTORY urdf launch meshes rviz worlds env
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
