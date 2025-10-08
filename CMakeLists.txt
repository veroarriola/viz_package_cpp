cmake_minimum_required(VERSION 3.8)
project(viz_package_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(viz_node src/viz_node.cpp)
add_executable(move_node src/move_node.cpp)
add_executable(ps2_joy_node src/ps2_joy_node.cpp)
add_executable(ps2_control_node src/ps2_control_node.cpp)

target_include_directories(viz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(viz_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(move_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(move_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ps2_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(ps2_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(viz_node rclcpp visualization_msgs std_msgs)
ament_target_dependencies(move_node rclcpp sensor_msgs geometry_msgs std_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(ps2_joy_node rclcpp sensor_msgs)
ament_target_dependencies(ps2_control_node rclcpp sensor_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
	viz_node
	move_node
  ps2_joy_node
  ps2_control_node
  DESTINATION lib/${PROJECT_NAME})

# Install
install(DIRECTORY
  launch    # launch files
  urdf      # robot models
  rviz      # RViz2 configuration file
  DESTINATION share/${PROJECT_NAME}
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
