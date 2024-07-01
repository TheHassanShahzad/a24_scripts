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

    # Node for micro_ros_agent
    micro_ros_agent_node = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
        output='screen'
    )

    # Node for rplidar_node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True
        }]
    )

    return LaunchDescription([
        odom_publisher_node,
        micro_ros_agent_node,
        rplidar_node
    ])
