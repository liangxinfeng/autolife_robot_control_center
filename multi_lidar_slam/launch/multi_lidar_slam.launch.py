import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pi = '3.141592653589793'
    open_rviz = LaunchConfiguration('open_rviz')
    config_dir = os.path.join(
        get_package_share_directory('multi_lidar_slam'),
        'config'
    )
    declare_open_rviz = DeclareLaunchArgument('open_rviz', default_value='true')

    front_translation = ['0.215', '0.0', '0.2']
    rear_translation = ['-0.215', '0.0', '0.2']
    # 根据实测遮挡结果，两个雷达当前在 base_link 下的前后方向是反的。
    # 这里先把两个雷达的 yaw 补偿对调，翻转各自的前向轴。
    front_rotation_rpy = [3.141592653589793, 0.0, 3.141592653589793]
    rear_rotation_rpy = [3.141592653589793, 0.0, 0.0]
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(open_rviz),
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        declare_open_rviz,
        # 两个雷达均为倒装；根据当前实测结果，前后朝向补偿已对调。
        # static_transform_publisher 的欧拉角参数顺序是 yaw pitch roll。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_front',
            arguments=front_translation + [pi, '0', pi, 'base_link', 'front_laser_frame']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_rear',
            arguments=rear_translation + ['0', '0', pi, 'base_link', 'rear_laser_frame']
        ),
        
        # Cartographer 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'multi_lidar_cloud.lua',
                '-start_trajectory_with_default_topics'
            ],
            remappings=[
                ('scan_1', '/topic_gv_front_lidar_0_307'),
                ('scan_2', '/topic_gv_rear_lidar_0_307'),
                ('odom', '/topic_gv_wheel_odom_0_307'),
            ],
            output='screen'
        ),
        
        # 占用网格节点延后启动，避免在子图列表未稳定时请求旧子图。
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_occupancy_grid_node',
                    name='cartographer_occupancy_grid_node',
                    parameters=[
                        {'use_sim_time': False},
                        {'resolution': 0.05},
                        {'publish_period_sec': 1.0}
                    ]
                ),
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[rviz_node]
        ),
    ])
