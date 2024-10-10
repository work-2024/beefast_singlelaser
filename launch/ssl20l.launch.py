#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  # parameters:  angle value unit is radian
  #              range value uint is meter
  ldlidar_node = Node(
    package='ldlidar',
    executable='ldlidar',
    name='ssl20l',
    output='screen',
    parameters=[
      {'product_type': 'SSL20L'},
      {'port_name': '/dev/ttyUSB0'},
      {'topic_name': 'scan'},
      {'frame_id': 'base_laser'}
    ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ssl20l',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld