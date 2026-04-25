#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_turtlebot3_gazebo = get_package_share_directory('my_robot_controller')
    pkg_turtlebot3_nav2 = get_package_share_directory('turtlebot3_navigation2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = os.path.join(
        get_package_share_directory('my_robot_controller'),
        'map',
        'map2.yaml'
    )
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )


    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'map': map_dir}.items()
    )
     


    goal_pose_publisher = Node(
           package='my_robot_controller',
           executable='navigation',
           name='navigation'
           
           
    )
   
   
    ld = LaunchDescription()
    ld.add_action(navigation_node)
    ld.add_action(gazebo_world)
    ld.add_action(goal_pose_publisher)


    return ld