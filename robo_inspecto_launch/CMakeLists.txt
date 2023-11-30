cmake_minimum_required(VERSION 3.5)
project(robo_inspecto_launch)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# compile xaacro files to urdf files in build directory
file(GLOB xacro_files CONFIGURE_DEPENDS urdf/*.xacro)

foreach(xacro_file ${xacro_files})
  get_filename_component(xacro_filename ${xacro_file} NAME_WE)
  xacro_add_xacro_file(${xacro_file} ${CMAKE_CURRENT_BINARY_DIR}/urdf/${xacro_filename}.urdf)
  list(APPEND xacro_outputs ${XACRO_OUTPUT_FILE})
endforeach()

# install urdf files
add_custom_target(xacro_target ALL DEPENDS ${xacro_outputs})
xacro_install(xacro_target ${xacro_outputs} DESTINATION urdf)

install(
  DIRECTORY launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()