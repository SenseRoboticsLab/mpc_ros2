cmake_minimum_required(VERSION 3.5)
project(mpc_ros)

# Compile as C++17 (recommended for ROS 2)
add_definitions(-std=c++17 -O3)

# Options for build configuration
option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
#find_package(costmap_2d REQUIRED)
#find_package(base_local_planner REQUIRED)
#find_package(move_base_msgs REQUIRED)

# Include directories
#include_directories(include)
#include_directories(/usr/include)
#include_directories(/usr/include/linux)
#include_directories(/usr/include/eigen3)
#link_directories(/usr/lib)

# Add executable
add_executable(tracking_reference_trajectory
        src/trackRefTraj.cpp
        src/trackRefTrajNodeRos2.cpp
)

# Link libraries
ament_target_dependencies(tracking_reference_trajectory
        rclcpp
        geometry_msgs
        nav_msgs
        std_msgs
        tf2
        tf2_ros
        visualization_msgs
        pluginlib
)

target_link_libraries(tracking_reference_trajectory ipopt)
target_include_directories(tracking_reference_trajectory PUBLIC
        include
        /usr/include/eigen3)

# Install targets
install(TARGETS tracking_reference_trajectory
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/mpc_trajectory_generation_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
