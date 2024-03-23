import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, LogInfo,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,OnProcessIO, OnProcessStart, OnShutdown)


def generate_launch_description():
    pkg_name = "hunter_sim_pkg"
    bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory(pkg_name)
    
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
    
    datum_params = os.path.join(pkg_dir,"config","datum_config.yaml")
    datumgen = Node(
        package=pkg_name,
        executable="DatumGen",
        output="log",
        parameters=[datum_params]
    )
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')]),
        launch_arguments={
            "use_sim_time" : use_sim_time,
        }.items(),
    )

    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )
    
    rviz_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(bringup_dir,"launch","rviz_launch.py")]
        ),
        condition=IfCondition(use_nav2_rviz),
    )
    
    return LaunchDescription([
        declare_sim_time,
        declare_rviz_nav2,
        simulators,
        localization,
        navigation2,
        rviz_nav2
    ])
    
