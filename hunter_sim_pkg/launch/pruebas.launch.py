import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess,DeclareLaunchArgument,IncludeLaunchDescription,TimerAction,LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command,NotSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    
    #OS stuff
    pkg_name = "hunter_sim_pkg"
    pkg_dir = get_package_share_directory(pkg_name)
    launch_dir = os.path.join(pkg_dir,'launch') 
    madgick_dir = get_package_share_directory("imu_complementary_filter")
    madgick_launch = os.path.join(madgick_dir,"launch","complementary_filter.launch.py")
    nav2_params = os.path.join(pkg_dir,"config", "nav2_no_map.yaml")
    bringup_dir = get_package_share_directory('nav2_bringup')

    #########################################################################################################################################
                                                        ######Launch parameters######
    use_nav2 = LaunchConfiguration('use_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
                                                    ######Parameters Declarations######
    declare_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='true:=Nav2 rviz config file, false:= Test rviz config file')
    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time, if true we start Gazebo')
    ##########################################################################################################################################
    
    #Start simulators (Rviz, Gazebo), including robot_state_publisher, if sim_time = true, we start Gazebo
    simulators = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir,'simulators_launch.launch.py')]),
        launch_arguments={
            'use_rviz': NotSubstitution(use_nav2),
            'use_sim_time':use_sim_time,
        }.items(),
    )
    
    #Get a Local Orientation from an Imu that doesn't have it (calculated from accelerometer and gyroscope)
    madwick_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([madgick_launch])
    )
    
    #Start Localization, launching two Extended Kalman filter and a NavSat transform node, IMPORTANT!! (Check sim_time or wont work at all)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir,"dual_ekf_navsat.launch.py")]),
        launch_arguments={
            "use_sim_time":use_sim_time
        }.items(),
    )
    

    #Custom node,Datum Generator, as we dont have a Magnetomter we need the offset respect to ENU system ( East = 0º and ANTICLOCKWISE)
    #Datum Generator also provides, if set in params_file, an IMU message with the sum of the madgick filter + Datum
    datum_params = os.path.join(pkg_dir,"config","datum_config.yaml")
    datumgen = Node(
        package=pkg_name,
        executable="DatumGen",
        output="screen",
        parameters=[datum_params],
        prefix="terminator -x"
    )
    
    #A node to receive Mqtt petitions and remaps to std_msgs::msg::String topic
    mqtt_params = os.path.join(get_package_share_directory("mqtt_client"),"config","mqtt_config.yaml")
    mqtt_client = Node(
        package="mqtt_client",
        executable="mqtt_client",
        name="mqtt_client",
        output="screen",
        arguments=["--params-file:= ",mqtt_params]
    )
    # mqtt_client = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("mqtt_client"),"launch","standalone.launch.ros2.xml")
    #     )
    # )
    
    #Custom node, that receives the Strings from de mqtt bridge, calculates the apropiate goal / goal_array and sends it to the commander,
    #also provides to the broker with the actual position of the robot
    string2ros_params = os.path.join(pkg_dir,"config","string2ros_config.yaml")
    
    string2ros = Node(
        package=pkg_name,
        executable="String2Ros",
        name="String2Ros",
        output="screen",
        parameters=[string2ros_params]
    )
    
    #Custom node, that receives either a goal petitions and send, a GoToPose petitions to Nav2, a goal_array and sends it to Waypointsfollower
    # or a follow me order where it sends  a GoToPose petitions but with a different Behaviour tree (BT)
    nav2_commander = Node(
        package="mqqt_to_nav2_commander",
        executable="send_poses",
        name="nav2_commander"
    )
    
    #   Nav2 Bringup with custom configuration, using Smac Hybrid A* as planner and MPPI as controller, prepared for ackerman model (REEDS-SHEEP)
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True) 
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": "True",
            "log_level":"warn"
        }.items(),
        condition=IfCondition(use_nav2),
    )
    
    # Rviz launch, only if nav2 is on, otherwise a custom configuration is launched from Simulators.launch
    rviz_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(bringup_dir,"launch","rviz_launch.py")]
        ),
        condition=IfCondition(use_nav2),
    )
    
    #####################################################################################
    #######TIMERS 2º AND 3º ARE EXTREMELY IMPORTANT, DO NOT LOWER THEM BY ANY CASE#######
    #####################################################################################
    
    
    #   5º timer to be launch, 3 second = "same as the last one"
    navigation2_timer = TimerAction(
                        period=3.0,
                        actions=[LogInfo(msg=' FINALLY, NAVIGATION2 STARTING...'),navigation2,rviz_nav2]
    )
    #   4º timer to be launch, 5 second = Just to not saturate with initialization of nodes (can be bring down or eliminate)
    interfaces_mqqt_timer = TimerAction(
                        period=5.0,
                        actions=[LogInfo(msg=' MQTT, STRING2ROS AND COMMANDER STARTING...'),navigation2_timer]
    )
    #   3º timer to be launch, 10 seconds = datumGen makes the robot move, IT is very important that when localization is launch
    #   the robot is completly stoped in its final position, otherwise will be inconsistencies in position/ orientation
    localization_timer = TimerAction(
                        period=10.0,
                        actions=[LogInfo(msg='LOCALIZATION STARTING...'),localization,interfaces_mqqt_timer]
    )
    #   2º timer to be launch, 9 seconds = madgick_filter NEEDS at least 5s to eliminate the noise of the IMU, 9s seems reasonable
    datum_timer = TimerAction(
                        period=9.0,
                        actions=[LogInfo(msg='DATUM STARTING...'),datumgen,localization_timer]
    )
    # 1º timer to be launch, 5 second = to make sure RSP and/or Gazebo ar ON
    madgick_timer =  TimerAction(
                        period=5.0,
                        actions=[LogInfo(msg='MADGICK STARTING...'),madwick_filter,datum_timer]
    )
    

    
    ld = LaunchDescription()


    ld.add_action(declare_nav2_cmd)
    ld.add_action(declare_sim_time_cmd)
    
    ld.add_action(simulators)
    ld.add_action(madgick_timer)
    
    return ld

    