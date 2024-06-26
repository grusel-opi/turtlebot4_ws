from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    namespace = EnvironmentVariable('ROBOT_NAMESPACE', default_value='')

    pkg_oakd = get_package_share_directory('oakd')

    cam_A_launch_file = PathJoinSubstitution(
        [pkg_oakd, 'launch', 'cam_A.launch.py'])
    cam_B_launch_file = PathJoinSubstitution(
        [pkg_oakd, 'launch', 'cam_B.launch.py'])

    actions = [
            PushRosNamespace(namespace),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([cam_A_launch_file]),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([cam_B_launch_file]),
            ),
        ]

    cam_rig_actions = GroupAction(actions)

    ld = LaunchDescription()
    ld.add_action(cam_rig_actions)
    return ld
