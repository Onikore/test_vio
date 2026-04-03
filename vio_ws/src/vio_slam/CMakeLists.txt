cmake_minimum_required(VERSION 3.14)
project(vio_slam VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(octomap REQUIRED)

# DBoW3 (may need manual find if not installed via package manager)
find_package(DBoW3 QUIET)
if(NOT DBoW3_FOUND)
    message(STATUS "DBoW3 not found, will attempt to build from source or use fallback")
endif()

# spdlog
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    # Header-only, can be added manually
endif()

# Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

# Source files
set(VIO_SLAM_SOURCES
    src/camera/camera_model.cpp
    src/camera/pinhole_camera.cpp
    src/camera/pinhole_radtan_camera.cpp
    src/camera/equidistant_camera.cpp
    src/camera/omni_camera.cpp
    src/camera/stereo_rig.cpp
    
    src/frontend/feature_tracker.cpp
    src/frontend/keyframe_manager.cpp
    src/frontend/exposure_controller.cpp
    
    src/imu/imu_preintegration.cpp
    src/imu/ekf_propagator.cpp
    src/imu/imu_initializer.cpp
    
    src/backend/sliding_window_optimizer.cpp
    src/backend/pose_graph_optimizer.cpp
    src/backend/marginalization.cpp
    
    src/loop_closure/loop_detector.cpp
    src/loop_closure/relocalization.cpp
    
    src/mapping/sparse_map.cpp
    src/mapping/occupancy_grid_builder.cpp
    
    src/utils/math_utils.cpp
    src/utils/config.cpp
    src/utils/thread_safe_queue.cpp
)

# Create library
add_library(vio_slam SHARED ${VIO_SLAM_SOURCES})

target_link_libraries(vio_slam
    ${OpenCV_LIBS}
    ceres
    sophus
    octomap
    ${DBoW3_LIBRARIES}
)

target_include_directories(vio_slam PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Unit tests
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    
    add_executable(vio_slam_test
        test/test_camera_models.cpp
        test/test_imu_preintegration.cpp
        test/test_sliding_window.cpp
        test/test_dbow3.cpp
    )
    
    target_link_libraries(vio_slam_test
        vio_slam
        GTest::gtest_main
    )
    
    add_test(NAME VioSlamTests COMMAND vio_slam_test)
endif()

# Installation
install(TARGETS vio_slam
    EXPORT vio_slam_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/vio_slam
    DESTINATION include
)

# Export targets
install(EXPORT vio_slam_targets
    FILE vio_slamTargets.cmake
    NAMESPACE vio_slam::
    DESTINATION lib/cmake/vio_slam
)

message(STATUS "VIO-SLAM core library configured")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  Ceres version: ${CERES_VERSION}")
