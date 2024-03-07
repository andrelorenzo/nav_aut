import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node

def generate_launch_description():
    
    bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory("hunter_sim_pkg")
    
    launch_dir = os.path.join(pkg_dir, 'launch')
    config_dir = os.path.join(pkg_dir, "config")
    nav2_params = os.path.join(config_dir, "nav2_no_map.yaml")
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav2_rviz = LaunchConfiguration('use_rviz_nav2')
    
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time')
    
    declare_rviz_nav2 = DeclareLaunchArgument(
        "use_rviz_nav2",
        default_value='true',
        description="Wether to use own rviz or nav2 rviz default configurations"
    )
    
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    ) 
    
    simulators = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir,'simulators_launch.launch.py')]),
        launch_arguments={
            'use_rviz': "False",
        }.items(),
    )
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')]),
    )

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": "true",
            "log_level": "warn",
        }.items(),
    )
    
    rviz_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(bringup_dir,"launch","rviz_launch.py")]
        ),
        condition=IfCondition(use_nav2_rviz),
    )
    twist_stamper = Node(
              package='twist_stamper',
              executable='twist_stamper',
              parameters=[{'use_sim_time': use_sim_time}],
              remappings=[('/cmd_vel_in','/cmd_vel'),
                          ('/cmd_vel_out','/cmd_vel_stamped')]
           )
    
    ld = LaunchDescription()

    ld.add_action(declare_sim_time)
    ld.add_action(declare_rviz_nav2)
    
    ld.add_action(simulators)
    ld.add_action(localization)
    ld.add_action(navigation2)
    ld.add_action(rviz_nav2)
    ld.add_action(twist_stamper)
    return ld
