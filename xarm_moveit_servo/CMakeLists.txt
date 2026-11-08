cmake_minimum_required(VERSION 3.5)
project(xarm_moveit_servo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(THIS_PACKAGE_LINK_TARGETS
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${control_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(xarm_joystick_input SHARED src/xarm_joystick_input.cpp)
target_link_libraries(xarm_joystick_input
  PUBLIC
  ${THIS_PACKAGE_LINK_TARGETS}
)
# rclcpp_components_register_nodes(xarm_joystick_input "xarm_moveit_servo::JoyToServoPub")
rclcpp_components_register_node(xarm_joystick_input PLUGIN "xarm_moveit_servo::JoyToServoPub" EXECUTABLE xarm_joystick_input_node)

add_executable(xarm_keyboard_input
  src/xarm_keyboard_input.cpp
)
target_link_libraries(xarm_keyboard_input
  PRIVATE
  ${THIS_PACKAGE_LINK_TARGETS}
)


install(TARGETS
  xarm_joystick_input
  xarm_joystick_input_node
  xarm_keyboard_input
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
