cmake_minimum_required(VERSION 3.8)
project(haptic_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies

set(DEPS
  ament_cmake
  ament_cmake_python
  hardware_interface
  rclcpp
  rclpy
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  interactive_markers
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  pantograph_library
  eigen3_cmake_module
  Eigen3
)

foreach(Dependency IN ITEMS ${DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  haptic_controller_parameters
  src/haptic_controller_parameters.yaml
)
add_library(${PROJECT_NAME} SHARED
  src/haptic_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  haptic_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "HAPTIC_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface haptic_controller_plugin.xml)

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

#-----------------------------------------------------
#   Cpp setup
#-----------------------------------------------------

# Include Cpp "include" directory
# include_directories(include)

# Create Cpp executables
add_executable(error_estimation src/error_estimation.cpp)
ament_target_dependencies(error_estimation PUBLIC ${DEPS})

# Install Cpp executables
install(
  TARGETS
  error_estimation
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
  haptic_controller
  haptic_controller_parameters
  EXPORT export_haptic_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

#-----------------------------------------------------
#   Install Python package
#-----------------------------------------------------
ament_python_install_package(
  ${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/markers.py
    scripts/trajectory_markerPY.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_haptic_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})
ament_package()
