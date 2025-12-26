cmake_minimum_required(VERSION 3.8)
project(obstacle_sim)

# --------------------
# Core build system
# --------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# --------------------
# ROS 2 dependencies
# --------------------
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# If you have custom services
find_package(rosidl_default_generators REQUIRED)

# Gazebo (for plugins / headers if needed)
# find_package(gazebo REQUIRED)

# --------------------
# Include directories
# --------------------
include_directories(
  include
  rvo_lib
  #${gazebo_INCLUDE_DIRS}
)

# --------------------
# Custom service (ROS 2 way)
# --------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGoals.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# --------------------
# RVO shared library
# --------------------
add_library(rvolib SHARED
  rvo_lib/nav_rvo.cpp
  rvo_lib/RVOSimulator.cpp
  rvo_lib/Agent.cpp
  rvo_lib/KdTree.cpp
  rvo_lib/Obstacle.cpp
)

ament_target_dependencies(rvolib
  rclcpp
)

# --------------------
# C++ nodes
# --------------------
add_executable(obstacle_sim_node src/obstacle_sim.cpp)
ament_target_dependencies(obstacle_sim_node
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(obstacle_sim_node
  rvolib
)

add_executable(set_goals_client src/set_goals_client.cpp)
ament_target_dependencies(set_goals_client
  rclcpp
  geometry_msgs
)

add_executable(formation_node src/formation.cpp)
ament_target_dependencies(formation_node
  rclcpp
  geometry_msgs
)

# --------------------
# Install C++ targets
# --------------------
install(
  TARGETS
    rvolib
    obstacle_sim_node
    set_goals_client
    formation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# --------------------
# Python nodes
# --------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  obstacle_sim/obstacle_sim.py
  obstacle_sim/spawn_cylinder.py
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------
# Finalize package
# --------------------
ament_package()
