import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    pkg_name = "hunter_sim_pkg"
    pkg_dir = get_package_share_directory(pkg_name)    
    datum_params = os.path.join(pkg_dir,"config","datum_config.yaml")
    
    datumgen = Node(
        package=pkg_name,
        executable="DatumGen",
        output="screen",
        name="Datumgen"
        parameters=[datum_params]
    )

    ld = LaunchDescription()

    ld.add_action(datumgen)

    return ld