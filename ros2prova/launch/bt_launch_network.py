from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='check_network_skill',
            executable='check_network_skill',
            output='screen',
            arguments=['--ros-args','--log-level', 'debug']
        ),
        # Node(
        #     package='start_services_skill',
        #     executable='start_services_skill',
        #     output='screen',
        #     arguments=['--ros-args','--log-level', 'debug']
        # ),
        # Node(
        #     package='alarm_battery_low_skill',
        #     executable='alarm_battery_low_skill',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', 'debug']
        # ),
        # Node(
        #     package='battery_drainer_skill',
        #     executable='battery_drainer_skill',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', 'debug']
        # ),
        # Node(
        #     package='battery_drainer_component',
        #     executable='battery_drainer_component',
        #     arguments=[('__log_level:=debug')]
        # ),
        # Node(
        #     package='battery_level_skill',
        #     executable='battery_level_skill',
        #     arguments=[('__log_level:=debug')]
        # ),
        # Node(
        #     package='battery_drainer_component',
        #     executable='battery_drainer_component',
        #     arguments=[('__log_level:=debug')]
        # ),
        Node(
            package='bt_executable',
            executable='bt_executable',
            arguments=[ '/workspaces/convince_simple_bts/ros2prova/BT/simple_bt.xml',('__log_level:=debug')]
        )
    ])

