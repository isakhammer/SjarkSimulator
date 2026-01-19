cmake_minimum_required(VERSION 3.20)

project(na_msg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
  "msg/Custom.msg"
  "msg/BsplinePath.msg"
  "msg/ControllerState.msg"
  "msg/BoatState.msg"
  "msg/Path.msg"
  "srv/AddTwoInts.srv"
  "srv/SetBool.srv"
  "srv/Trigger.srv"
  DEPENDENCIES std_msgs
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
