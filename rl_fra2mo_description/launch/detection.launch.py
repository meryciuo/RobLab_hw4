from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


 
def generate_launch_description():
    
    declared_arguments = []

    path1=get_package_share_directory('aruco_ros')
    
    path=os.path.join(path1,"launch","single.launch.py")


    navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('rl_fra2mo_description'),
                                    'launch',
                                    'fra2mo_explore.launch.py'])]),
    )

    aruco_dect = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
    )
    
    nodes_to_start = [
        navigation,
        aruco_dect,
    ]

    return LaunchDescription(nodes_to_start)
    