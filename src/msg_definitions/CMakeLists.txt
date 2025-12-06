cmake_minimum_required(VERSION 3.20)

project(msg_definitions)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
  "msg/Float64.msg"
  "srv/AddTwoInts.srv"
  "srv/SetBool.srv"
  "srv/Trigger.srv"
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
