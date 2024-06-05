import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler,DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    package_name='diff_robot'

    pkg_bringup = get_package_share_directory(package_name)

    rplidar_arg = DeclareLaunchArgument(
            'include_rplidar',
            default_value='True',
            description='Indicates whether to include rplidar launch.')
    
    rplidar =  LaunchConfiguration('include_rplidar')
     # Include rplidar launch file
    include_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'rplidar.launch.py'),
        ),
        launch_arguments={
            "serial_port": '/dev/ttyUSB_LIDAR',
        }.items(),
                condition=IfCondition(rplidar)
    )


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )


    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)},
                    controller_params_file],
        remappings=[
            ('/diff_cont/cmd_vel', '/cmd_vel'), # Used if use_stamped_vel param is true
            ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'), # Used if use_stamped_vel param is false
            ('/diff_cont/cmd_vel_out', '/cmd_vel_out'), # Used if publish_limited_velocity param is true
            ('/diff_cont/odom', '/odom'),
        ]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont","--controller-manager", "/controller_manager"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad","--controller-manager", "/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    rplidar_timer = TimerAction(period=3.0, actions=[include_rplidar])


    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rplidar_arg,
        rplidar_timer,
    ])