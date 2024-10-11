#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # LDROBOT LiDAR publisher node
    # parameters:  angle value unit is radian
    #              range value uint is meter
    lifecycle_node = LifecycleNode(
        package='beefast_single_laser',
        executable='single_laser',
        name='lifecycle_single_laser',
        namespace='',
        output='screen',
        parameters=[
            {'product_type': 'SSL20N'},
            {'port_name': '/dev/ttyUSB_linelaser'},
            {'topic_name': 'laser'},
            {'frame_id': 'base_laser'}
        ],
    )

     # 创建第二个生命周期节点
    pub_distance_node = LifecycleNode(
        package='beefast_single_laser',
        executable='pub_distance',
        name='lifecycle_pub_distance',
        namespace='',
        output='screen',
    )

    # 创建第三个生命周期节点
    pub_cliff_detection_node = LifecycleNode(
        package='beefast_single_laser',
        executable='cliff_detection',
        name='lifecycle_cliff_detection',
        namespace='',
        output='screen',
        parameters=[
            {'LimitDistance': 0.5},
        ],
    )


    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ssl20n',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser']
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Correctly add lifecycle_node instead of the undefined ldlidar_node
    ld.add_action(lifecycle_node)
    ld.add_action(pub_distance_node)
    ld.add_action(pub_cliff_detection_node)
    ld.add_action(base_link_to_laser_tf_node)

    return ld
