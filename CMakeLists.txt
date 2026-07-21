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
find_package(diagnostic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

include_directories(include)

add_executable(hound_ll_control_node
  src/hound_ll_control_node.cpp
  src/ll_controller_node.cpp
)
ament_target_dependencies(hound_ll_control_node
  rclcpp
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  mavros_msgs
  vesc_msgs
  ackermann_msgs
)
target_compile_features(hound_ll_control_node PUBLIC cxx_std_17)

install(TARGETS hound_ll_control_node
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

# console_scripts in setup.py are ignored by ament_python_install_package
# (see mavros/CMakeLists.txt). Install explicit wrappers instead.
install(PROGRAMS
  scripts/bag_replay
  scripts/clipseg_mask_node
  scripts/sam_refine_node
  scripts/wheel_odom_node
  scripts/hal_monitor_node
  scripts/wait_for_topic_hz.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
