# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    
    package_dir = get_package_share_directory("sim_robot")
    launch_dir = os.path.join(package_dir, 'launch')
    params_dir = os.path.join(package_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map.yaml")
    
    world = os.path.join(package_dir, "worlds", "gazebo_world.world")
    
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )


    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Specify the RSP Action
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_dir,'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true','use_ros2_control':'false'}.items()
    )
    
    # Specify the Gz Action
    # start_gazebo_server_cmd = ExecuteProcess(
    #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so', world],
    #     cwd=[launch_dir], output='both')

    # start_gazebo_client_cmd = ExecuteProcess(
    #     cmd=['gzclient'],
    #     cwd=[launch_dir], output='both')
    
    
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo_world.launch.py'))
    )
    
    localization_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_dir, 'launch', 'dual_ekf_navsat.launch.py')]),
             )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

 
    # Create the launch description and populate
    ld = LaunchDescription()

    # simulator launch
    ld.add_action(gazebo_cmd)
    
    # rsp launch
    ld.add_action(rsp)
    
    # robot localization launch
    ld.add_action(localization_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # rviz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)

    return ld