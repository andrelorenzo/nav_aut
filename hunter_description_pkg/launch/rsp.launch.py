import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration,Command
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import xacro
from launch.conditions import IfCondition


def generate_launch_description():
    
    pkg_dir = get_package_share_directory("hunter_description_pkg")
    urdf_hunter_file = os.path.join(pkg_dir,'urdf','hunter.urdf.xacro')
    
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    ros2_ctrl_cmd = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="True",
        description="Wether to use ros2 control plugins"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Wether to use Simulation time (GAZEBO only)"
    )
    
    robot_description_config = Command(['xacro ',urdf_hunter_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    # Launch Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',  
        parameters=[params],
        )
    
    ld = LaunchDescription()
    ld.add_action(ros2_ctrl_cmd)
    ld.add_action(sim_time_cmd)
    
    ld.add_action(rsp)
    
    return ld