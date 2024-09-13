cmake_minimum_required(VERSION 3.16)
project(deepdrive_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Specify the required version of ros2_control
find_package(controller_manager 2.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 2.0.0 or higher is required.")
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  deepdrive_control
  SHARED
  hardware/controller_plugin.cpp
  hardware/motor.cpp
)

target_compile_features(deepdrive_control PUBLIC cxx_std_17)
target_include_directories(deepdrive_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/deepdrive_control>
)
ament_target_dependencies(
  deepdrive_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DEEPDRIVE_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface deepdrive_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/deepdrive_control
)

install(
  DIRECTORY launch config urdf
  DESTINATION share/deepdrive_control
)
install(TARGETS deepdrive_control
  EXPORT export_deepdrive_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)

#   ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
#   ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
# endif()

## EXPORTS
ament_export_targets(export_deepdrive_control HAS_LIBRARY_TARGET)

ament_export_libraries(
  deepdrive_control
  # fake_robot
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
