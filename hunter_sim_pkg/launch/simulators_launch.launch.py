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
    urdf = os.path.join(pkg_dir, 'description', 'hunter2.xacro')
    
    xacro_file = os.path.join(pkg_dir,'description','simple_urdf','robot.urdf.xacro')
    
    
    coppelia_scene = os.path.join(pkg_dir,'scenes','ros2_mapirlab_hunter20.ttt')
    launch_dir = os.path.join(pkg_dir,'launch')
    gz_world_dir = os.path.join(pkg_dir, "scenes", "gazebo_world.world")
    namespace = ""
    
    
    
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
    
    # Create variable with robot_description-> robot_state_publisher
    robot_description = xacro.process_file(
        os.path.join(pkg_dir, 'description','hunter2.xacro'),
        mappings={'frame_ns': namespace}
        )
    robot_description = robot_description.toprettyxml(indent='  ')

    robot_description_simple = Command(
        ['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time]
        )
    
    params = {'robot_description': robot_description_simple, 'use_sim_time': use_sim_time}
    # Launch Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        #parameters=[{'robot_description': robot_description}],
        parameters=[params],
        #arguments=[urdf],
        )
    
    # coppelia = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(
    #             pkg_dir,'launch','coppelia_Sim.xml'
    #         ),
    #         condition=IfCondition(use_coppelia)
    #     )
    # )
    gz_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', gz_world_dir],
        cwd=[launch_dir], output='both')

    gz_client= ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hunter20',
                                   '-z','10'],
                        output='screen')  
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )
    ack_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont"]    
    )
    
    ld = LaunchDescription()
    
    #launch declares
    ld.add_action(ddeclare_rviz_cmd)
    ld.add_action(declare_sim_time_cmd)
    ld.add_action(declare_ros2_control)
    
    #launch simulators
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    
    ld.add_action(rviz)
    ld.add_action(rsp)
    
    ld.add_action(spawn_entity)
    ld.add_action(joint_broad_spawner)
    ld.add_action(ack_cont_spawner)
    
    
    return ld

    