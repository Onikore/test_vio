cmake_minimum_required(VERSION 3.14)
project(vio_slam_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)

# Find vio_slam core library
find_package(vio_slam REQUIRED)

# Include directories
include_directories(
    include
    ${OCTOMAP_INCLUDE_DIRS}
)

# ROS2 node executable
add_executable(vio_slam_node
    src/vio_slam_node.cpp
)

target_link_libraries(vio_slam_node
    vio_slam::vio_slam
)

ament_target_dependencies(vio_slam_node
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    message_filters
    octomap_ros
    octomap_msgs
)

# Install targets
install(TARGETS vio_slam_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Install RViz config
install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
