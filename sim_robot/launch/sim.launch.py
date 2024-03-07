import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():


    package_name='sim_robot'
    rviz = LaunchConfiguration('rviz',default=True)

    # Launch Robot_State_Publisher launcher (sim_time := true)
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true','use_ros2_control':'false'}.items()
    )
    
    world_file_name = 'gazebo_world.world'
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)
  
    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
    

    # Launch Gazebo simulation itself
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazibo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                                      'world': world}.items()
             )
    
    # Node of gazebo package to spawn robot in gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')    
    
    rviz_launch = Node(package='rviz2',
                       executable='rviz2',
                       arguments=['-d','src/sim_robot/config/rviz_test_robot.rviz','use_sim_time','true'],
                       condition=IfCondition(rviz))

    
    
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
    return LaunchDescription([
        declare_world_cmd,
        rsp,
        gazebo,
        #joystick,
        spawn_entity,
        #gui_ctl,
        rviz_launch,
        ack_cont_spawner,
        joint_broad_spawner,
        #ros_bridge_websocket,
    ])