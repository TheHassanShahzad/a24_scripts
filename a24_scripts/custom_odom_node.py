import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from math import pi, sin, cos

class EncoderToOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_to_odom_node')
        # Define constants and initial values
        self.wheel_diameter = 0.07
        self.wheel_spacing = 0.226
        self.counts_per_rev = 987

        self.right_encoder_counts = 0
        self.left_encoder_counts = 0
        self.right_wheel_displacement = 0
        self.left_wheel_displacement = 0

        self.pose_pos_x = 0.0
        self.pose_pos_y = 0.0
        self.pose_pos_z = 0.0
        self.pose_ori_x = 0.0
        self.pose_ori_y = 0.0
        self.pose_ori_z = 0.0
        self.pose_ori_w = 1.0
        self.twist_lin_x = 0.0
        self.twist_ang_z = 0.0

        self.euler_theta = 0.0

        self.old_time = 0
        self.current_time = 0
        self.old_right_wheel_displacement = 0
        self.old_left_wheel_displacement = 0
        self.old_euler_theta = 0

        self.subscription = self.create_subscription(
            Int64MultiArray,
            '/encoder_data',
            self.encoder_callback,
            10)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Setting up the Odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_footprint'

        # Setting initial pose and twist values
        self.odom_msg.pose.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        self.odom_msg.twist.twist = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )

    def encoder_callback(self, msg):
        # For now, we are not using the encoder data, just publishing a simple odom message
        # self.get_logger().info(f'Received encoder data: {msg.data}')
        self.right_encoder_counts, self.left_encoder_counts = msg.data
        self.right_wheel_displacement, self.left_wheel_displacement = self.get_wheels_displacement(self.wheel_diameter, self.counts_per_rev, self.right_encoder_counts, self.left_encoder_counts)

        # get x, y position of robot and how much it has rotated about z axis
        self.get_robot_position(self.wheel_spacing, self.right_wheel_displacement, self.left_wheel_displacement)
        # print(self.pose_pos_x, self.pose_pos_y, self.euler_theta)

        self.pose_ori_z = self.get_quaternion(self.euler_theta)[2]
        self.pose_ori_w = self.get_quaternion(self.euler_theta)[3]

        self.current_time = (self.get_clock().now().to_msg().sec*(10**9))+self.get_clock().now().to_msg().nanosec

        d_time = (self.current_time - self.old_time)/(10**9)
        # print(d_time)
        if d_time != self.current_time:
            d_right_wheel_displacement = self.right_wheel_displacement - self.old_right_wheel_displacement
            d_left_wheel_displacement= self.left_wheel_displacement - self.old_left_wheel_displacement
            d_euler_theta = self.euler_theta - self.old_euler_theta

            right_wheel_velocity = d_right_wheel_displacement/d_time
            left_wheel_velociy = d_left_wheel_displacement/d_time

            self.twist_lin_x = (right_wheel_velocity + left_wheel_velociy)/2
            self.twist_ang_z = d_euler_theta/d_time

        self.old_time = self.current_time
        self.old_right_wheel_displacement = self.right_wheel_displacement
        self.old_left_wheel_displacement = self.left_wheel_displacement
        self.old_euler_theta = self.euler_theta

        #updating odom
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose = Pose(
            position=Point(x=self.pose_pos_x, y=self.pose_pos_y, z=self.pose_pos_z),
            orientation=Quaternion(x=self.pose_ori_x, y=self.pose_ori_y, z=self.pose_ori_z, w=self.pose_ori_w)
        )
        self.odom_msg.twist.twist = Twist(
            linear=Vector3(x=self.twist_lin_x, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.twist_ang_z)
        )

        # Publish the odom message
        self.publisher.publish(self.odom_msg)

    def get_wheels_displacement(self, wheel_diameter, counts_per_rev, right_encoder_counts, left_encoder_counts):
        right_wheel_displacement= (wheel_diameter/2)*((right_encoder_counts/counts_per_rev)*(2*pi))
        left_wheel_displacement = (wheel_diameter/2)*((left_encoder_counts/counts_per_rev)*(2*pi))

        return [right_wheel_displacement, left_wheel_displacement]
    
    def get_robot_position(self, wheel_spacing, right_wheel_displacement, left_wheel_displacement):
        dc = (right_wheel_displacement+left_wheel_displacement)/2
        theta = (right_wheel_displacement - left_wheel_displacement)/wheel_spacing
        x_coord = dc*cos(theta)
        y_coord = dc*sin(theta)

        self.pose_pos_x, self.pose_pos_y, self.euler_theta = [x_coord, y_coord, theta]

    def get_quaternion(self, euler_theta):
        return [0, 0, sin(euler_theta/2), cos(euler_theta/2)]
    
def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
