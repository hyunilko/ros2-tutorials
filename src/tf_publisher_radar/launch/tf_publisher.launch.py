import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):

    urdf_model = LaunchConfiguration('urdf_model').perform(context)

    return [
        # :x: rviz2
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
            namespace='rviz2',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("publish_tf").perform(context)),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])}]
        )
    ]

def generate_launch_description():
    package_path = get_package_share_directory("tf_publisher_radar")

    declared_arguments = [
        DeclareLaunchArgument("use_rviz", default_value='true'),
        DeclareLaunchArgument("publish_tf", default_value='true'),
        # DeclareLaunchArgument("rviz_config", default_value=os.path.join(package_path, "rviz", "unified_radars.rviz")),
        # DeclareLaunchArgument("urdf_model", default_value=os.path.join(package_path, "urdf", "unified_radars.urdf")),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(package_path, "rviz", "multi_radars_each_pub.rviz")),
        DeclareLaunchArgument("urdf_model", default_value=os.path.join(package_path, "urdf", "multi_radars_each_pub_jig.urdf")),
    ]    # multi_radars_each_pub multi_radars_each_pub_jig

 
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
