import os

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# # for using mesh import in urdf
# pkg_share_path = os.pathsep + os.path.join(get_package_prefix("aidin_m1"), 'share')
# if 'GAZEBO_MODEL_PATH' in os.environ:
#     os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
# else:
#     os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path


def generate_launch_description():

    robot_file_name ="aidin_m1.urdf"
    pkg_name = "aidin_m1"

    pkg_path = get_package_share_directory(pkg_name)
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    urdf = os.path.join(pkg_path, "urdf", robot_file_name)

    # Start Gazebo server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={"world": "/usr/share/gazebo-11/worlds/empty.world",
                          "verbose": "true"}.items()
    )

    # Start Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        parameters=[],
        arguments=["-entity", "aidin_m1", "-file", urdf],
        output="screen",
    )

    # Start joint_control_node
    gazebo_aidin_m1_control = Node(
        package="aidin_m1",
        executable="aidin_m1_main",
        output="screen",
    )

    return LaunchDescription(
        [
            start_gazebo_server,
            start_gazebo_client,
            spawn_entity,
            gazebo_aidin_m1_control,

        ]
    )
