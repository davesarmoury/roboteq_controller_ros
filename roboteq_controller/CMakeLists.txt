cmake_minimum_required(VERSION 3.5)
project(roboteq_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(
  roboteq_control
  SHARED
  src/roboteq_controller_node.cpp
)

target_compile_features(roboteq_control PUBLIC cxx_std_17)
target_include_directories(roboteq_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/roboteq_control>
)

ament_target_dependencies(
  roboteq_control PUBLIC
  rclcpp
  std_msgs
  rclcpp_components
  rcutils
  serial
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface roboteq_control_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/roboteq_control
)

install(TARGETS roboteq_control
  EXPORT export_roboteq_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_targets(export_roboteq_control HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  rclcpp_components
  rcutils
  serial
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
