import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription,DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,Command
from launch.conditions import IfCondition
import xacro
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_dir = get_package_share_directory("hunter_sim_pkg")
    rviz_file = os.path.join(pkg_dir, 'rviz', 'default_rviz1.rviz')
    
    urdf_hunter_file = os.path.join(pkg_dir,'description','simple_urdf','robot.urdf.xacro')
    sdf_hunter_file = os.path.join(pkg_dir,'models','model1','model.sdf')
    with open(sdf_hunter_file, 'r') as infp:
        robot_desc = infp.read()
        
    
    launch_dir = os.path.join(pkg_dir,'launch') 
    gz_world_dir = os.path.join(pkg_dir, "scenes", "gazebo_world.world")
    namespace = "hunter20"
    
    
    
    ddeclare_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time')
    
    declare_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Whether to use ros2_control')
    
    
    
    # Launch rviz2 with a predefined configuration file
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d' + rviz_file],
        condition=IfCondition(use_rviz)
    )
   
    robot_description_config = Command(['xacro ', urdf_hunter_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    # Launch Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        
        parameters=[params]
        )

    gz_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', gz_world_dir],
        cwd=[launch_dir], output='both')

    gz_client= ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

     
    
    spawn_entity = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', "hunter", 
                        '-topic', 'robot_description',
                        '-z', "10",],
                        output='screen')
     
    ack_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont"]    
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )
    
    
    ld = LaunchDescription()
    
    #launch declares
    ld.add_action(ddeclare_rviz_cmd)
    ld.add_action(declare_sim_time_cmd)
    ld.add_action(declare_ros2_control)
    
    ld.add_action(rsp)
    #launch simulators
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    
    #launch Rviz
    ld.add_action(rviz)
    
    #launch spawners
    ld.add_action(spawn_entity)
    ld.add_action(ack_cont_spawner)
    ld.add_action(joint_broad_spawner)
    
    
    return ld

    