from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='/tb15',
        description='Robot namespace'
    )
]

def generate_launch_description():
    package_name = 'my_tb4_nav'
    package_directory = get_package_share_directory(package_name)
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
