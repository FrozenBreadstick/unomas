cmake_minimum_required(VERSION 3.5)
project(field_sampling)

set(CMAKE_BUILD_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
 
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
 
# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()
 
# Embed RPATH so detector can find its shared libraries at runtime
set(CMAKE_BUILD_RPATH "$ORIGIN")
 
# Required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleMsg.msg"
  DEPENDENCIES geometry_msgs
)
 
# Shared library: object processing
add_library(obj_proc_lib src/object_processing.cpp)
ament_target_dependencies(obj_proc_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
 
# Shared library: lidar detection
add_library(lidar_detect src/lidar_detection.cpp)
ament_target_dependencies(lidar_detect
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
target_link_libraries(lidar_detect obj_proc_lib)
 
# Executable: detector node
add_executable(detector src/main.cpp)
ament_target_dependencies(detector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
target_link_libraries(detector
  lidar_detect
)
 
# Install targets
install(TARGETS
  detector
  obj_proc_lib
  lidar_detect
  DESTINATION lib/${PROJECT_NAME}
)
 
# Optional: install data files (commented out)
# install(DIRECTORY data DESTINATION share/${PROJECT_NAME})
 
# Finalize package
ament_package()