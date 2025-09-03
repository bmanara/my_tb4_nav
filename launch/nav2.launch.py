from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [get_package_share_directory('my_tb4_nav'), 'config', 'nav2_config.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for the navigation stack'
    ),
    DeclareLaunchArgument(
        'namespace',
        default_value='/tb6',
        description='Robot namespace'
    )
]

def launch_setup(context, *args, **kwargs):
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    nav2_launch = PathJoinSubstitution([
        pkg_nav2_bringup, 
        'launch', 
        'navigation_launch.py'
    ])

    namespace_str = namespace.perform(context)
    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(namespace_str + '/global_costmap/scan', namespace_str + '/scan'),
        SetRemap(namespace_str + '/local_costmap/scan', namespace_str + '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments=[
                ('namespace', namespace),
                ('params_file', nav2_params.perform(context)),
                ('use_composition', 'False'),
                ('use_sim_time', 'False')
            ]
        )
    ])

    return [nav2]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
