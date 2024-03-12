import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration,Command
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,ExecuteProcess
import xacro
from launch.conditions import IfCondition

#   This launch file has been created to launch a very descriptive urdf file for the hunter20 robot,
#   has been done by Andre L. Torres in the MAPIR department of robotics for the Uiversity of Malaga
#   as the final proyect of Enginireeng in robotics, electronics and Mechatronics.

def generate_launch_description():
    
    ## DIRECTORY DECLARATION
    pkg_dir = get_package_share_directory("hunter_description_pkg")
    launch_dir = os.path.join(pkg_dir,'launch')
    
    
    
    ##  RVIZ
    rviz_file = os.path.join(pkg_dir, 'rviz', 'default_rviz1.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d' + rviz_file],
    )
    
    
    ##  GAZEBO
    gz_world_dir = os.path.join(pkg_dir,'worlds','gazebo_world.world')
    gz_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', gz_world_dir],
        cwd=[launch_dir], output='both')

    gz_client= ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

     
    ## SPAWNER ROBOT
    spawn_entity = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', "hunter", 
                        '-topic', 'robot_description',
                        '-z', "10",],
                        output='screen')
     
    
    ## SPAWNER CONTROLLER
    ack_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont"]    
    )
    
    
    ##  SPAWNER JOINT BROAD
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )
    
    ld = LaunchDescription()
    
    # ld.add_action(rviz)
    
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    
    ld.add_action(spawn_entity)
    ld.add_action(ack_cont_spawner)
    ld.add_action(joint_broad_spawner)
    return ld