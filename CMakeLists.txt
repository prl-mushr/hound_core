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
find_package(mavros_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(libmavconn REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavlink REQUIRED)

include_directories(include)

# In-process RealSense + cuVSLAM C++ API (cuvslam2.h).
# Isaac ROS debs ship the C API only; install the Thor/CUDA13 C++ SDK from
# https://github.com/nvidia-isaac/cuVSLAM/releases (or override these paths).
if(NOT DEFINED CUVSLAM_ROOT)
  set(CUVSLAM_ROOT "/opt/cuvslam")
endif()
set(CUVSLAM_INCLUDE_DIR "${CUVSLAM_ROOT}/include")
set(CUVSLAM_LIB "${CUVSLAM_ROOT}/bin/libcuvslam.so")
if(NOT EXISTS "${CUVSLAM_INCLUDE_DIR}/cuvslam/cuvslam2.h")
  message(FATAL_ERROR
    "cuvslam2.h not found under ${CUVSLAM_INCLUDE_DIR}. "
    "Install the cuVSLAM C++ SDK to ${CUVSLAM_ROOT} or pass -DCUVSLAM_ROOT=...")
endif()

add_executable(realsense_cuvslam_node
  src/realsense_cuvslam_node.cpp
)
target_include_directories(realsense_cuvslam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CUVSLAM_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
)
ament_target_dependencies(realsense_cuvslam_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(realsense_cuvslam_node
  ${realsense2_LIBRARY}
  ${CUVSLAM_LIB}
)
set_target_properties(realsense_cuvslam_node PROPERTIES
  BUILD_RPATH "${CUVSLAM_ROOT}/bin"
  INSTALL_RPATH "${CUVSLAM_ROOT}/bin"
)
target_compile_features(realsense_cuvslam_node PUBLIC cxx_std_17)

# Modular A/B twin: CameraDevice + VslamBackend (same ROS surface).
add_executable(realsense_cuvslam_modular_node
  src/realsense_cuvslam_modular_main.cpp
  src/realsense_cuvslam_modular_node.cpp
  src/realsense_camera_device.cpp
  src/cuvslam_backend.cpp
)
target_include_directories(realsense_cuvslam_modular_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CUVSLAM_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
)
ament_target_dependencies(realsense_cuvslam_modular_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(realsense_cuvslam_modular_node
  ${realsense2_LIBRARY}
  ${CUVSLAM_LIB}
)
set_target_properties(realsense_cuvslam_modular_node PROPERTIES
  BUILD_RPATH "${CUVSLAM_ROOT}/bin"
  INSTALL_RPATH "${CUVSLAM_ROOT}/bin"
)
target_compile_features(realsense_cuvslam_modular_node PUBLIC cxx_std_17)

# --- FCU mini-router: MAVLink + EKF + LL (replaces mavros+ekf+ll when enabled) ---
if(NOT DEFINED INERTIAL_NAV_SHIM)
  set(INERTIAL_NAV_SHIM "${CMAKE_CURRENT_SOURCE_DIR}/../Inertial_nav_shim")
endif()
set(INERTIAL_NAV_ROS2 "${CMAKE_CURRENT_SOURCE_DIR}/../inertial_nav_ros2")

add_executable(hound_fcu_control_modular_node
  src/hound_fcu_control_modular_main.cpp
  src/hound_fcu_control_modular_node.cpp
  src/mavlink_bridge.cpp
  src/ekf_runner.cpp
  src/ll_runner.cpp
  src/ll_controller_registry.cpp
  src/pid.cpp
  src/ackermann_ll_controller.cpp
  src/holonomic_ll_controller.cpp
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
  ${mavlink_INCLUDE_DIRS}
)
target_compile_definitions(hound_fcu_control_modular_node PRIVATE NOMINMAX)
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
  mavros_msgs
  vesc_msgs
  ackermann_msgs
  libmavconn
  tf2
  tf2_geometry_msgs
)
target_compile_features(hound_fcu_control_modular_node PUBLIC cxx_std_17)

install(TARGETS
  realsense_cuvslam_node
  realsense_cuvslam_modular_node
  hound_fcu_control_modular_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config dora
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

# console_scripts in setup.py are ignored by ament_python_install_package
# (see mavros/CMakeLists.txt). Install explicit wrappers instead.
install(PROGRAMS
  scripts/bag_replay
  scripts/dora_clipseg_encoder
  scripts/dora_seg_refine
  scripts/dora_yolo_world
  scripts/wheel_odom_node
  scripts/hal_monitor_node
  scripts/hw_latency_probe
  scripts/wait_for_topic_hz.py
  scripts/realsense_cuvslam_stereo
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
    mavros_msgs
    libmavconn
    tf2
    tf2_geometry_msgs
  )
endif()

ament_package()
