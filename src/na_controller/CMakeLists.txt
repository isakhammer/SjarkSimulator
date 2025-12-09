cmake_minimum_required(VERSION 3.20)
project(na_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment thhe following section in order to fill in
# further dependencies manually.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msg_definitions REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(throttle src/throttle.cpp)
target_link_libraries(throttle PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${msg_definitions_TARGETS})

install(TARGETS
  throttle
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
