cmake_minimum_required(VERSION 3.8)
project(robocup_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Include cpp "include" directory
include_directories(include)

# Create cpp executables
add_executable(cpp_exe src/cpp_node.cpp)
ament_target_dependencies(cpp_exe rclcpp)

# Install cpp executables
install(TARGETS
  cpp_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  scripts/py_node.py
  scripts/circle.py
  scripts/omnidrive.py
  scripts/nav_to_pose.py
  scripts/detect_marker.py
  scripts/camera.py
  scripts/follow_marker.py
  scripts/flip_laser.py
  scripts/odom.py
  scripts/odom_frame_broadcast.py
  benchmark-planners/scripts/process_data.py
  benchmark-planners/scripts/metric.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch robotino3_description src rcll_sim benchmark-planners maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
