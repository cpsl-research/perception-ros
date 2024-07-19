import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fov_config = os.path.join(
        get_package_share_directory("fov_estimator"),
        "config",
        "lidar_concave_hull.yaml",
    )

    fov_node = Node(
        package="fov_estimator",
        executable="lidar_concave_hull",
        name="fov_estimator",
        parameters=[fov_config],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([fov_node])
