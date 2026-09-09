cmake_minimum_required(VERSION 3.8)
project(multi_lidar_slam)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

# 雷达融合节点
add_executable(lidar_fusion_node src/lidar_fusion_node.cpp)
ament_target_dependencies(lidar_fusion_node
  message_filters
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
)

# 安装
install(TARGETS lidar_fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})

ament_package()
