cmake_minimum_required(VERSION 3.8)
project(control_center_slam)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(map_save_panel SHARED
  include/${PROJECT_NAME}/map_save_panel.hpp
  src/map_save_panel.cpp
)
set_target_properties(map_save_panel PROPERTIES
  AUTOMOC ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)
target_include_directories(map_save_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_save_panel
  ament_index_cpp
  cartographer_ros_msgs
  pluginlib
  rclcpp
  rviz_common
)
target_link_libraries(map_save_panel Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

install(DIRECTORY config launch maps rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES rviz_plugins.xml DESTINATION share/${PROJECT_NAME})

install(TARGETS map_save_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mapping_scan_adapter.py
  scripts/mapping_keyboard_teleop.py
  scripts/slam_ui_node.py
  scripts/motion_safety_watchdog.py
  scripts/incremental_mapping.py
  DESTINATION lib/${PROJECT_NAME}
)

# This private executable is statically linked against the patched SLAM core.
# Never replace /opt/ros or globally overlay cartographer for other applications.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/runtime/cartographer_multi_origin_node" OR
   NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/runtime/cartographer_occupancy_grid_node")
  message(FATAL_ERROR "Run scripts/build_multi_origin_cartographer.sh first; private mapping executables are missing.")
endif()
install(PROGRAMS runtime/cartographer_multi_origin_node
  runtime/cartographer_occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rviz_common)
ament_package()
