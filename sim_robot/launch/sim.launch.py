import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    package_name='sim_robot'

    # Launch Robot_State_Publisher launcher (sim_time := true)
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true','se_ros2_control':'false'}.items()
    )

    # Launch Gazebo simulation itself
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazibo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    
    # Node of gazebo package to spawn robot in gazibo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')

    joystick = IncludeLaunchDescription(
                 PythonLaunchDescriptionSource([os.path.join(
                     get_package_share_directory(package_name), 'launch', 'joystick.launch.py')]),
                     launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    #gui_ctl = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', output='screen')
    
    rviz_launch = Node(package='rviz2',
                       executable='rviz2',
                       arguments=['-d','src/sim_robot/config/rviz_test_robot.rviz','use_sim_time','true']
    )
    
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
    ros_bridge_websocket = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        arguments=['use_sim_time'],
    )
    return LaunchDescription([
        rsp,
        gazebo,
        joystick,
        spawn_entity,
        #gui_ctl,
        rviz_launch,
        ack_cont_spawner,
        joint_broad_spawner,
        ros_bridge_websocket,
    ])