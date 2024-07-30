import os

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_file_name ="aidin_m1.urdf"
    pkg_name = "aidin_m1"

    pkg_path = get_package_share_directory(pkg_name)
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    urdf = os.path.join(pkg_path, "urdf", robot_file_name)

    aidin_m1_description_share = FindPackageShare(package='aidin_m1').find('aidin_m1')
    default_model_path = os.path.join(aidin_m1_description_share, 'urdf/aidin_m1.urdf')

    # Custom world path
    custom_world_path = os.path.join(pkg_path, "worlds", "house.world")

    # Start Gazebo server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={"world": custom_world_path,
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
            DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='~/dev_ws/src/aidin_m1/meshes'),
            start_gazebo_server,
            start_gazebo_client,
            spawn_entity,
            gazebo_aidin_m1_control,

        ]
    )
