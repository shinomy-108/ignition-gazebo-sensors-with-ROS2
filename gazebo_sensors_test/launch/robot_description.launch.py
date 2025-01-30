from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # パッケージのパスを取得
    pkg_name = "gazebo_sensors_test"
    pkg_share = get_package_share_directory(pkg_name)

    # xacroファイルのパス
    urdf_file = os.path.join(pkg_share, "urdf", "rrbot.xacro")

    # Robot State Publisher の設定
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["xacro ", urdf_file])}],
    )

    return LaunchDescription([robot_state_publisher_node])
