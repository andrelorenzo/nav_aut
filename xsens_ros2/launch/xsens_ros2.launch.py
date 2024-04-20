import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory("xsens_ros2")
    params_file = os.path.join(pkg_dir,"params","xsens_params.yaml")
    xsens_driver = Node(
                    package=pkg_dir,
                    executable='ros2_driver',
                    name='ros2_driver',
                    output='screen',
                    parameters=[params_file]
                )
    
    ld = LaunchDescription()
    ld.add_action(xsens_driver)
    return ld

    