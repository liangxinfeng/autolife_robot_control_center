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
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(filters REQUIRED)
add_library(wall_tail_filter SHARED src/wall_tail_filter.cpp)
target_compile_features(wall_tail_filter PRIVATE cxx_std_17)
ament_target_dependencies(wall_tail_filter filters sensor_msgs pluginlib)
pluginlib_export_plugin_description_file(filters wall_tail_plugins.xml)
install(TARGETS wall_tail_filter LIBRARY DESTINATION lib)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_amcl REQUIRED)
add_executable(amcl_startup_compat src/amcl_startup_compat.cpp)
ament_target_dependencies(amcl_startup_compat nav2_amcl rclcpp)
install(TARGETS amcl_startup_compat RUNTIME DESTINATION lib/${PROJECT_NAME})
add_library(settled_mppi SHARED src/settled_mppi.cpp)
target_compile_features(settled_mppi PRIVATE cxx_std_17)
ament_target_dependencies(settled_mppi nav2_mppi_controller nav2_controller nav2_core
  nav2_costmap_2d pluginlib tf2 tf2_geometry_msgs nav_msgs)
pluginlib_export_plugin_description_file(nav2_core settled_plugins.xml)
install(TARGETS settled_mppi LIBRARY DESTINATION lib)
add_library(exact_goal_planner SHARED src/exact_goal_planner.cpp)
target_compile_features(exact_goal_planner PRIVATE cxx_std_17)
ament_target_dependencies(exact_goal_planner nav2_smac_planner nav2_core
  nav2_costmap_2d pluginlib tf2 tf2_geometry_msgs nav_msgs)
pluginlib_export_plugin_description_file(nav2_core navigation_plugins.xml)
install(TARGETS exact_goal_planner LIBRARY DESTINATION lib)

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
  scripts/head_rgb_panel.py
  scripts/charger_waypoints.py
  scripts/short_distance_servo.py
  scripts/pose_continuity.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
if(BUILD_TESTING)
  enable_testing()
  add_executable(exact_near_goal_regression test/exact_near_goal_regression.cpp)
  target_compile_features(exact_near_goal_regression PRIVATE cxx_std_17)
  target_include_directories(exact_near_goal_regression PRIVATE src)
  ament_target_dependencies(exact_near_goal_regression nav_msgs tf2_geometry_msgs)
  add_test(NAME exact_near_goal_regression COMMAND exact_near_goal_regression)
  add_executable(wall_tail_regression test/wall_tail_regression.cpp)
  target_compile_features(wall_tail_regression PRIVATE cxx_std_17)
  target_include_directories(wall_tail_regression PRIVATE src)
  target_compile_options(wall_tail_regression PRIVATE -UNDEBUG)
  add_test(NAME wall_tail_regression COMMAND wall_tail_regression)
  add_executable(amcl_startup_regression test/amcl_startup_regression.cpp)
  ament_target_dependencies(amcl_startup_regression nav2_amcl rclcpp)
  add_test(NAME amcl_startup_regression COMMAND amcl_startup_regression --ros-args
    --params-file ${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml)
  set_tests_properties(amcl_startup_regression PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=83;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" TIMEOUT 30)
  add_test(NAME amcl_lifecycle_regression COMMAND ${CMAKE_COMMAND} -E env
    AMCL_PROBE_EXECUTABLE=$<TARGET_FILE:amcl_startup_compat>
    /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/amcl_configure_probe.py
    ${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml)
  set_tests_properties(amcl_lifecycle_regression PROPERTIES TIMEOUT 45)
  find_package(behaviortree_cpp REQUIRED)
  add_executable(occupied_approach_bt_test test/occupied_approach_bt_test.cpp)
  ament_target_dependencies(occupied_approach_bt_test behaviortree_cpp)
  add_test(NAME occupied_approach_bt_test COMMAND occupied_approach_bt_test
    ${CMAKE_CURRENT_SOURCE_DIR}/config/occupied_approach.xml)
  add_executable(occupied_approach_planner_test test/occupied_approach_planner_test.cpp)
  target_compile_features(occupied_approach_planner_test PRIVATE cxx_std_17)
  ament_target_dependencies(occupied_approach_planner_test nav2_smac_planner nav2_core
    nav2_costmap_2d pluginlib tf2 tf2_geometry_msgs nav_msgs)
  add_test(NAME occupied_approach_planner_test COMMAND occupied_approach_planner_test)
  set_tests_properties(occupied_approach_planner_test PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=88" TIMEOUT 20)
  add_executable(settled_checker_regression test/settled_checker_regression.cpp)
  target_compile_features(settled_checker_regression PRIVATE cxx_std_17)
  target_include_directories(settled_checker_regression PRIVATE src)
  ament_target_dependencies(settled_checker_regression nav2_mppi_controller nav2_controller nav2_core
    nav2_costmap_2d pluginlib tf2 tf2_geometry_msgs nav_msgs)
  add_test(NAME settled_checker_regression COMMAND settled_checker_regression)
  set_tests_properties(settled_checker_regression PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=88" TIMEOUT 10)
  add_executable(arrival_settle_regression test/arrival_settle_regression.cpp)
  target_include_directories(arrival_settle_regression PRIVATE src)
  add_test(NAME arrival_settle_regression COMMAND arrival_settle_regression)
  add_executable(terminal_approach_regression test/terminal_approach_regression.cpp)
  target_compile_features(terminal_approach_regression PRIVATE cxx_std_17)
  target_include_directories(terminal_approach_regression PRIVATE src)
  ament_target_dependencies(terminal_approach_regression nav_msgs tf2 tf2_geometry_msgs)
  add_test(NAME terminal_approach_regression COMMAND terminal_approach_regression)
  add_executable(exact_goal_endpoint_regression test/exact_goal_endpoint_regression.cpp)
  target_include_directories(exact_goal_endpoint_regression PRIVATE src)
  ament_target_dependencies(exact_goal_endpoint_regression nav_msgs)
  add_test(NAME exact_goal_endpoint_regression COMMAND exact_goal_endpoint_regression)
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
