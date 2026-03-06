cmake_minimum_required(VERSION 3.8)
project(wfd_explorer)

# -------------------------------------------------------------------------
# Compiler settings
# -------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(fmt REQUIRED)

# -------------------------------------------------------------------------
# Core (ROS-free) library
# -------------------------------------------------------------------------
add_library(wfd_core STATIC
  src/wfd_processor.cpp
)

target_include_directories(wfd_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# wfd_core only needs fmt and rclcpp logging headers (for ROSLogger)
ament_target_dependencies(wfd_core
  rclcpp   # for rclcpp/logging.hpp used in logger.hpp
)
target_link_libraries(wfd_core fmt::fmt)

# -------------------------------------------------------------------------
# ROS interface library
# -------------------------------------------------------------------------
add_library(wfd_ros_interface STATIC
  src/ros_interface.cpp
)

target_include_directories(wfd_ros_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(wfd_ros_interface
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(wfd_ros_interface wfd_core fmt::fmt)

# -------------------------------------------------------------------------
# Executable
# -------------------------------------------------------------------------
add_executable(wfd_explorer_node src/main.cpp)

target_include_directories(wfd_explorer_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(wfd_explorer_node wfd_ros_interface wfd_core fmt::fmt)

ament_target_dependencies(wfd_explorer_node
  rclcpp
)

# -------------------------------------------------------------------------
# Install
# -------------------------------------------------------------------------
install(TARGETS wfd_explorer_node wfd_core wfd_ros_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------------------
# Tests
# -------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
