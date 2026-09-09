cmake_minimum_required(VERSION 3.8)
project(control_center_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Isolated Jazzy 1.3.10 docking server with the upstream reverse-projection fix.
# Keep /opt/ros and vendor services untouched; reuse installed controller/plugins.
find_package(opennav_docking REQUIRED)
set(docking_dependencies angles rclcpp rclcpp_action rclcpp_lifecycle
  rclcpp_components std_msgs sensor_msgs visualization_msgs nav2_graceful_controller
  nav2_util nav2_msgs nav_msgs geometry_msgs builtin_interfaces tf2_ros
  tf2_geometry_msgs pluginlib yaml_cpp_vendor opennav_docking_core)
foreach(dependency IN LISTS docking_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(yaml-cpp REQUIRED)
find_library(docking_controller_library NAMES controller
  HINTS ${opennav_docking_DIR}/../../../lib REQUIRED)
add_executable(docking_server_compat
  src/docking_compat/main.cpp src/docking_compat/docking_server.cpp
  src/docking_compat/dock_database.cpp src/docking_compat/navigator.cpp)
target_compile_features(docking_server_compat PRIVATE cxx_std_17)
target_include_directories(docking_server_compat PRIVATE ${opennav_docking_INCLUDE_DIRS})
ament_target_dependencies(docking_server_compat ${docking_dependencies})
target_link_libraries(docking_server_compat ${docking_controller_library} yaml-cpp::yaml-cpp)
install(TARGETS docking_server_compat RUNTIME DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DeleteWaypoint.srv"
  "srv/ListWaypoints.srv"
  "srv/NavigateToWaypoint.srv"
  "srv/NavigateToPoseRefined.srv"
  "srv/NavigateWaypointSequence.srv"
  "srv/RenameWaypoint.srv"
  "srv/SaveWaypoint.srv"
  "srv/SwitchWaypointMap.srv"
)

add_executable(cartographer_global_relocalization
  src/cartographer_global_relocalization.cpp
)
ament_target_dependencies(cartographer_global_relocalization
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
install(TARGETS
  cartographer_global_relocalization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/arm_control_service_wrapper.py
  scripts/waypoint_manager_node.py
  scripts/nav_cmd_vel_relay_node.py
  scripts/cmd_vel_arbiter_node.py
  scripts/keyboard_waypoint_control_node.py
  scripts/navigation_startup_on_initialpose_node.py
  scripts/navigation_status_overlay_node.py
  scripts/proximity_voice_alert_node.py
  scripts/odom_topic_relay_node.py
  scripts/navigation_ui_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  scripts/short_distance_servo.py
  scripts/pose_continuity.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
if(BUILD_TESTING)
  enable_testing()
  add_executable(docking_projection_regression test/docking_projection_regression.cpp)
  target_compile_features(docking_projection_regression PRIVATE cxx_std_17)
  target_include_directories(docking_projection_regression PRIVATE src/docking_compat)
  ament_target_dependencies(docking_projection_regression
    nav2_graceful_controller nav2_util geometry_msgs tf2 tf2_geometry_msgs)
  add_test(NAME docking_projection_regression COMMAND docking_projection_regression)
  add_test(NAME relocalization_entrypoint_smoke
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/entrypoint_smoke.py
      $<TARGET_FILE:cartographer_global_relocalization>)
  set_tests_properties(relocalization_entrypoint_smoke PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=88" TIMEOUT 30)
  add_executable(relocalization_regression test/relocalization_regression.cpp)
  ament_target_dependencies(relocalization_regression
    geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs
    tf2 tf2_geometry_msgs tf2_ros)
  add_test(NAME relocalization_regression COMMAND relocalization_regression
    --ros-args -p auto_start:=false -p publish_initial_pose:=false)
  set_tests_properties(relocalization_regression PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=88" TIMEOUT 120)
  foreach(variant current baseline)
    add_executable(relocalization_benchmark_${variant} test/offline_benchmark.cpp)
    ament_target_dependencies(relocalization_benchmark_${variant}
      geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs
      tf2 tf2_geometry_msgs tf2_ros)
  endforeach()
  target_compile_definitions(relocalization_benchmark_baseline PRIVATE RELOCALIZATION_BASELINE)
endif()
ament_package()
