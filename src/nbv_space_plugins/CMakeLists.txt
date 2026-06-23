cmake_minimum_required(VERSION 3.8)
project(nbv_space_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Gz Harmonic deps 
# gz-sim8  = Gazebo Sim (Harmonic)
# gz-plugin2 = plugin registration macros
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Plugin shared library
add_library(SpacecraftPosePlugin SHARED
  src/SpacecraftPosePlugin.cc
)

# Gz Harmonic requires C++17
target_compile_features(SpacecraftPosePlugin PUBLIC cxx_std_17)

target_link_libraries(SpacecraftPosePlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
)

ament_target_dependencies(SpacecraftPosePlugin
  rclcpp
  geometry_msgs
)

# Install the .so where Gazebo can find it (GZ_SIM_SYSTEM_PLUGIN_PATH is
# extended automatically via the env-hook below once you source setup.bash)
install(TARGETS SpacecraftPosePlugin
  LIBRARY DESTINATION lib
)

# Enviornment hook
# Sourcing /ws/install/setup.bash will automatically prepend this package's
# lib/ to GZ_SIM_SYSTEM_PLUGIN_PATH, so Gazebo finds SpacecraftPosePlugin
# without any manual export.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in"
)

ament_package()