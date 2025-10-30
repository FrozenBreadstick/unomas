cmake_minimum_required(VERSION 3.8)
project(unomas)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

find_package(geometry_msgs REQUIRED)


# custom messages
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StrNum.msg"
  "msg/StrStr.msg"
  "msg/StatusUpdatePacket.msg"
  "srv/DebugTrigger.srv"
  "srv/DebugResponse.srv"
  "srv/StatusUpdateService.srv"
  DEPENDENCIES geometry_msgs
)

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
