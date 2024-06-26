from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Node for odom_publisher
    odom_publisher_node = Node(
        package='a24_scripts',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen'
    )

    # Node for pid_controller
    open_loop_controller_node = Node(
        package='a24_scripts',
        executable='open_loop_controller',
        name='open_loop_controller',
        output='screen'
    )

    # Node for micro_ros_agent
    micro_ros_agent_node = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
        output='screen'
    )

    return LaunchDescription([
        odom_publisher_node,
        open_loop_controller_node,
        micro_ros_agent_node
    ])
