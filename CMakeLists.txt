cmake_minimum_required(VERSION 3.8)
project(unomas)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(grid_map_core REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)


# custom messages
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StrNum.msg"
  "msg/StrStr.msg"
  "msg/StatusUpdatePacket.msg"
  "msg/SoilInfo.msg"

  "srv/DebugTrigger.srv"
  "srv/DebugResponse.srv"
  "srv/StatusUpdateService.srv"
  "srv/QuerySoil.srv"
  "srv/TerrainSoilData.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(custom_messages ${PROJECT_NAME} "rosidl_typesupport_cpp")


# all node executables
add_executable(base_station
  src/base_station/base_station_main.cpp
  src/base_station/base_station_status.cpp
  src/base_station/base_station_register.cpp
  src/base_station/base_station_terrain_soil.cpp
  src/base_station/base_station_path.cpp
)
target_link_libraries(base_station
  ${custom_messages}
)
ament_target_dependencies(base_station
  rclcpp
  geometry_msgs
)


add_executable(ui_bridge
  src/ui/ros/ui_main.cpp
  src/ui/ros/ui_bridge.cpp
)
target_link_libraries(ui_bridge
  ${custom_messages}
)
ament_target_dependencies(ui_bridge
  rclcpp
)

add_executable(simulation_extras
  src/simulation/simulation_main.cpp
  src/simulation/simulation_extra.cpp
)
target_link_libraries(simulation_extras
  ${custom_messages}
)
ament_target_dependencies(simulation_extras
  rclcpp
  grid_map_core
)

add_executable(robot
  src/robot/robot_main.cpp
  src/robot/robot_controller.cpp
  src/robot/robot_uplink.cpp
)
target_link_libraries(robot
  ${custom_messages}
)
ament_target_dependencies(robot
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

#install executables to lib
install(
  TARGETS 
    base_station
    ui_bridge
    simulation_extras
    robot
  DESTINATION lib/${PROJECT_NAME}
)

#install data files to share
install(
  DIRECTORY config launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
