cmake_minimum_required(VERSION 3.8)
project(assignment1_RT2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

add_library(action_server SHARED
  src/Action_Server.cpp
)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  action_tutorials_interfaces
  action_msgs
)
rclcpp_components_register_node(action_server
  PLUGIN "Action_Server"
  EXECUTABLE action_server_node
)

add_library(ui_node SHARED
  src/UI_Node.cpp
)
target_include_directories(ui_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ui_node
  rclcpp
  rclcpp_action
  rclcpp_components
  action_tutorials_interfaces
  action_msgs
)
rclcpp_components_register_node(ui_node
  PLUGIN "UI_Node"
  EXECUTABLE ui_node_exe
)
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

install(TARGETS
  action_server
  ui_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
