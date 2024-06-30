import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int64MultiArray
import tf2_ros
import math

WHEEL_BASE = 0.226  # Distance between the wheels
TICKS_PER_REVOLUTION = 990  # Adjust this based on your encoder specification
WHEEL_RADIUS = 0.035  # Adjust this based on your wheel size
ENCODER_THRESHOLD = 5  # Encoder counts threshold for velocity update
PUBLISH_INTERVAL = 0.001  # Publish interval in seconds

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.encoder_sub = self.create_subscription(
            Int64MultiArray,  # Subscribing to Int64MultiArray type
            'encoder_data',  # The topic name is 'encoder_counts'
            self.encoder_callback,
            10)

        self.timer = self.create_timer(PUBLISH_INTERVAL, self.publish_odometry)

        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time_nsec = self.get_clock().now().nanoseconds
        self.lin_x = 0.0
        self.ang_z = 0.0

        self.accumulated_left_encoder = 0
        self.accumulated_right_encoder = 0

    def publish_odometry(self):
        odom_quat = self.create_quaternion_from_yaw(self.theta)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.twist.twist.linear.x = self.lin_x
        odom.twist.twist.angular.z = self.ang_z

        self.odom_pub.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_quat

        self.tf_broadcaster.sendTransform(t)

    def encoder_callback(self, msg):
        right_encoder = msg.data[0]  # First element is the right wheel encoder count
        left_encoder = msg.data[1]  # Second element is the left wheel encoder count

        delta_left = left_encoder - self.last_left_encoder
        delta_right = right_encoder - self.last_right_encoder

        self.accumulated_left_encoder += delta_left
        self.accumulated_right_encoder += delta_right

        if abs(self.accumulated_left_encoder) >= ENCODER_THRESHOLD or abs(self.accumulated_right_encoder) >= ENCODER_THRESHOLD:
            delta_left_distance = (self.accumulated_left_encoder / TICKS_PER_REVOLUTION) * (2 * math.pi * WHEEL_RADIUS)
            delta_right_distance = (self.accumulated_right_encoder / TICKS_PER_REVOLUTION) * (2 * math.pi * WHEEL_RADIUS)
            
            delta_distance = (delta_left_distance + delta_right_distance) / 2.0
            delta_theta = (delta_right_distance - delta_left_distance) / WHEEL_BASE

            self.x += delta_distance * math.cos(self.theta + delta_theta / 2.0)
            self.y += delta_distance * math.sin(self.theta + delta_theta / 2.0)
            self.theta += delta_theta

            # Normalize theta to be within the range -pi to pi
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

            # Find odom twist
            time_now = self.get_clock().now().nanoseconds
            delta_time = abs(time_now - self.last_time_nsec) / (10**9)
            if delta_time > 0:
                self.lin_x = delta_distance / delta_time
                self.ang_z = delta_theta / delta_time
            self.last_time_nsec = time_now

            # Reset accumulated encoders
            self.accumulated_left_encoder = 0
            self.accumulated_right_encoder = 0

        self.last_left_encoder = left_encoder
        self.last_right_encoder = right_encoder

    def create_quaternion_from_yaw(self, yaw):
        quat = TransformStamped().transform.rotation
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
