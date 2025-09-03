from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml


PACKAGE_NAME = 'my_tb4_nav'

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='/tb6',
        description='Robot namespace'
    ),
    DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [FindPackageShare(PACKAGE_NAME), 'config', 'slam_config.yaml']
        )
    )
]

def generate_launch_description():
    package_directory = get_package_share_directory(PACKAGE_NAME)
    rviz_config = PathJoinSubstitution(
        [package_directory, 'rviz', 'nav.rviz']
    )

    namespace = LaunchConfiguration('namespace')

    rviz = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        )
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)

    return ld
