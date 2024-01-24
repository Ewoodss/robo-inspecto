cmake_minimum_required(VERSION 3.8)
project(robo_inspecto_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED)

add_library(
  robo_inspecto_hardware
  SHARED
  src/motor_hardware.cpp
)
target_include_directories(
  robo_inspecto_hardware
  PUBLIC
  include
)
ament_target_dependencies(
  robo_inspecto_hardware
  PUBLIC
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(
  robo_inspecto_hardware
  PUBLIC
  ${Boost_LIBRARIES}
)

# prevent pluginlib from using boost
target_compile_definitions(robo_inspecto_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface robo_inspecto_hardware.xml)

install(
  TARGETS
  robo_inspecto_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_motor_hardware test/test_motor_hardware.cpp)
  target_include_directories(test_motor_hardware PRIVATE include)
  ament_target_dependencies(
    test_motor_hardware
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  robo_inspecto_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
