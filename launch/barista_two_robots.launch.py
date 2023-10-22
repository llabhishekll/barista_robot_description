from os import environ
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    # gazebo launch description
    path_gazebo = Path(get_package_share_directory("gazebo_ros"))
    path_launch = path_gazebo / "launch" / "gazebo.launch.py"

    # package launch description
    path_root = Path(get_package_share_directory("barista_robot_description"))
    path_urdf = path_root / "xacro" / "barista_robot_model.urdf.xacro"
    path_rviz = path_root / "rviz" / "config.rviz"
    path_wrld = path_root / "worlds" / "empty.world"
    path_mesh = path_root / "meshes"

    # gazebo path
    environ["GAZEBO_MODEL_PATH"] += (
        ":" + path_root.parent.as_posix() + ":" + path_mesh.as_posix()
    )

    # parameters
    namespace1, robot_name1 = "rick", "rick_barista_robot"  # blue
    namespace2, robot_name2 = "morty", "morty_barista_robot"  # red

    # return launch
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path_launch.as_posix())
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=["-d", path_rviz.as_posix()],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher_node",
                namespace=namespace1,
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": True,
                        "frame_prefix": f"{namespace1}/",
                        "robot_description": Command(
                            [
                                " xacro ", path_urdf.as_posix(),
                                " namespace:=", namespace1,
                                " include_laser:=", "true",
                            ]
                        ),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher_node",
                namespace=namespace2,
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": True,
                        "frame_prefix": f"{namespace2}/",
                        "robot_description": Command(
                            [
                                " xacro ", path_urdf.as_posix(),
                                " namespace:=", namespace2,
                                " include_laser:=", "true",
                                " inverse:=", "true",
                            ]
                        ),
                    }
                ],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_entity",
                output="screen",
                arguments=[
                    "-entity", robot_name1,
                    "-x", "0", "-y", "0", "-z", "0",
                    "-R", "0", "-P", "0", "-Y", "0",
                    "-topic", f"{namespace1}/robot_description",
                ],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_entity",
                output="screen",
                arguments=[
                    "-entity", robot_name2,
                    "-x", "2", "-y", "2", "-z", "0",
                    "-R", "0", "-P", "0", "-Y", "0",
                    "-topic", f"{namespace2}/robot_description",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                namespace=namespace1,
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "world", f"{namespace1}/odom"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                namespace=namespace2,
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "world", f"{namespace2}/odom"],
            ),
        ]
    )
