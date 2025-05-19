cmake_minimum_required(VERSION 3.8)
project(adaptive_cruise_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB_RECURSE LIB_SOURCES "include/serialib/*.cpp" "include/serialib/*.c")
file(GLOB_RECURSE LIB_HEADERS "include/serialib/*.hpp" "include/serialib/*.h")

add_library(adaptive_cruise_control_component SHARED
  src/adaptive_cruise_control.cpp
  src/serialib.cpp
)

# Component olarak derlenmesi için eklemeler:
rclcpp_components_register_node(adaptive_cruise_control_component
  PLUGIN "adaptive_cruise_control::AdaptiveCruiseControl"
  EXECUTABLE adaptive_cruise_control
)

ament_target_dependencies(adaptive_cruise_control_component
  rclcpp
  rclcpp_components
  sensor_msgs
  laser_geometry
  Eigen3
  OpenCV
  cv_bridge
  vision_msgs
)




ament_export_targets(export_adaptive_cruise_control_component)
install(
  TARGETS adaptive_cruise_control_component
  EXPORT export_adaptive_cruise_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


target_include_directories(adaptive_cruise_control_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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

ament_package()
