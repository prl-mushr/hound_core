cmake_minimum_required(VERSION 3.8)
project(hound_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(libmavconn REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)

include_directories(include)

# --- FCU mini-router: MAVLink + EKF + LL (replaces mavros+ekf+ll when enabled) ---
# EKF source is pulled from nav_filter (Inertial_nav_shim submodule), not defined here.
if(NOT DEFINED INERTIAL_NAV_SHIM)
  set(INERTIAL_NAV_SHIM "${CMAKE_CURRENT_SOURCE_DIR}/../Inertial_nav_shim")
endif()
set(INERTIAL_NAV_ROS2 "${CMAKE_CURRENT_SOURCE_DIR}/../inertial_nav_ros2")
set(VESC_DRIVER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../vesc_ros2/vesc_driver")

add_executable(hound_fcu_control_modular_node
  src/hound_fcu_control_modular_main.cpp
  src/hound_fcu_control_modular_node.cpp
  src/mavlink_bridge.cpp
  src/ekf_runner.cpp
  src/ll_runner.cpp
  src/vesc_runner.cpp
  src/ll_controller_registry.cpp
  src/pid.cpp
  src/ackermann_ll_controller.cpp
  src/holonomic_ll_controller.cpp
  ${VESC_DRIVER_SRC}/src/vesc_interface.cpp
  ${VESC_DRIVER_SRC}/src/vesc_packet.cpp
  ${VESC_DRIVER_SRC}/src/vesc_packet_factory.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/estimator_22states.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/estimator_utilities.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/ekf_time.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/yaw_fusion.cpp
  ${INERTIAL_NAV_SHIM}/nav_filter/wmt_shim.cpp
  ${INERTIAL_NAV_ROS2}/src/frame_conversions.cpp
)
target_include_directories(hound_fcu_control_modular_node PRIVATE
  ${INERTIAL_NAV_SHIM}/nav_filter
  ${INERTIAL_NAV_ROS2}/include
  ${VESC_DRIVER_SRC}/include
  ${mavlink_INCLUDE_DIRS}
)
target_compile_definitions(hound_fcu_control_modular_node PRIVATE
  NOMINMAX
  BOOST_BIND_GLOBAL_PLACEHOLDERS
)
target_compile_options(hound_fcu_control_modular_node PRIVATE
  -Wno-unused-parameter
  -Wno-reorder
  -Wno-class-memaccess
)
ament_target_dependencies(hound_fcu_control_modular_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  std_msgs
  vesc_msgs
  ackermann_msgs
  libmavconn
  tf2
  tf2_geometry_msgs
  serial_driver
  io_context
)
# vesc_interface uses Boost (bind, shared_ptr, crc, function).
find_package(Boost REQUIRED COMPONENTS system)
target_link_libraries(hound_fcu_control_modular_node Boost::system)
target_compile_features(hound_fcu_control_modular_node PUBLIC cxx_std_17)

install(TARGETS
  hound_fcu_control_modular_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

# console_scripts in setup.py are ignored by ament_python_install_package.
install(PROGRAMS
  scripts/bag_replay
  scripts/wheel_odom_node
  scripts/hal_monitor_node
  scripts/hw_latency_probe
  scripts/wait_for_topic_hz.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid_safety test/test_pid_safety.cpp)
  target_include_directories(test_pid_safety PRIVATE include)
  target_compile_features(test_pid_safety PUBLIC cxx_std_17)
  ament_target_dependencies(test_pid_safety rclcpp)

  ament_add_gtest(test_holonomic_ll_controller
    test/test_holonomic_ll_controller.cpp
    src/holonomic_ll_controller.cpp
    src/ll_controller_registry.cpp
    src/mavlink_bridge.cpp
  )
  target_include_directories(test_holonomic_ll_controller PRIVATE
    include
    ${mavlink_INCLUDE_DIRS}
  )
  target_compile_features(test_holonomic_ll_controller PUBLIC cxx_std_17)
  target_compile_definitions(test_holonomic_ll_controller PRIVATE NOMINMAX)
  ament_target_dependencies(test_holonomic_ll_controller
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    diagnostic_msgs
    std_msgs
    libmavconn
    tf2
    tf2_geometry_msgs
  )
endif()

ament_package()
