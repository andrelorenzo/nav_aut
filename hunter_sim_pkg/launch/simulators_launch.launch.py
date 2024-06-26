import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess,DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_dir = get_package_share_directory("hunter_sim_pkg")
    rviz_file = os.path.join(pkg_dir, 'rviz', 'default_rviz1.rviz')
    
    urdf_hunter_file = os.path.join(pkg_dir,'description','simple_urdf','robot.urdf.xacro')
    
    launch_dir = os.path.join(pkg_dir,'launch') 
    gz_world_dir = os.path.join(pkg_dir, "scenes", "gazebo_world.world")
    namespace = "hunter20"
    
    ############################################
    ####    CONFIG FOR TURTLEBOT--TESTS   ####
    ############################################
    urdf_turtle_bot = os.path.join(pkg_dir, 'urdf', 'turtlebot3_waffle_gps.urdf')
    with open(urdf_turtle_bot, 'r') as infp:
        robot_description_turtlebot = infp.read()
    models_dir = os.path.join(pkg_dir, "models")
    models_dir += os.pathsep + \
        f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    set_gazebo_model_path_cmd = None
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + \
            os.pathsep + models_dir
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", gazebo_model_path)
    else:
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", models_dir)
    set_tb3_model_cmd = SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle")
    ############################################
    ############################################
    ############################################
    
    declare_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time')
    
    
    
    
    # Launch rviz2 with a predefined configuration file
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + rviz_file],
        output="screen",
        condition=IfCondition(use_rviz),
        prefix="terminator -x"
    )

   
    robot_description_config = Command(['xacro ', urdf_hunter_file, ' sim_mode:=', use_sim_time])
    # robot_description_config for hunter and robot_description_turtlebot for turtlebot
    params = {'robot_description': robot_description_turtlebot, 'use_sim_time': use_sim_time}
    # Launch Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        parameters=[params],
        )

    gazebo_params = os.path.join(pkg_dir,"config","gazebo_params.yaml")
    gazebo = ExecuteProcess( 
    cmd=['gazebo', gz_world_dir, '-s', 'libgazebo_ros_init.so',  
    '-s', 'libgazebo_ros_factory.so', '--ros-args', '--params-file', gazebo_params], 
    output='screen', 
    )
    # gz_server = ExecuteProcess(
    #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so',gz_world_dir," params_file:= ",gazebo_params],
    #     cwd=[launch_dir], 
    #     output="screen",
    #     condition = IfCondition(use_sim_time)
    #     )

    # gz_client= ExecuteProcess(
    #     cmd=['gzclient'],
    #     cwd=[launch_dir], output="log",
    #     condition = IfCondition(use_sim_time),
    #     )
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    #     ),
    #     launch_arguments={
    #         "world": gz_world_dir,
    #         'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params
    #     }.items(),
    #     condition = IfCondition(use_sim_time)
    # )

     
    
    spawn_entity = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', "hunter", 
                        '-topic', 'robot_description',
                        '-z', "10",],
                        output="screen"
                        )
     
    ack_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont"],
        output="screen"
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )
    
    
    ld = LaunchDescription()
    
    #launch declares
    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_tb3_model_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_sim_time_cmd)
    
    ld.add_action(rsp)
    #launch simulators
    ld.add_action(gazebo)
    # ld.add_action(gz_client)
    
    #launch Rviz
    ld.add_action(rviz)
    
    #launch spawners
    # ld.add_action(spawn_entity)
    # ld.add_action(ack_cont_spawner)
    # ld.add_action(joint_broad_spawner)
    
    
    return ld

    