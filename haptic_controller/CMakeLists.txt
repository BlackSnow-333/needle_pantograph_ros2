cmake_minimum_required(VERSION 3.8)
project(haptic_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(trajectory_msgs REQUIRED)

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
)

foreach(Dependency IN ITEMS ${DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()


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
include_directories(include)

# Create Cpp executable
add_executable(error_estimation src/error_estimation.cpp)
ament_target_dependencies(error_estimation PUBLIC ${DEPS})

# Install Cpp executables
install(TARGETS error_estimation
  DESTINATION lib/${PROJECT_NAME})


#-----------------------------------------------------
#   Install Python package
#-----------------------------------------------------
ament_python_install_package(
  ${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/markers.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
