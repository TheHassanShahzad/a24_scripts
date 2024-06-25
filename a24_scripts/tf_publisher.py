import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from nav_msgs.msg import Odometry
import tf2_ros
import math

class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_footprint'

        # Extract position
        transform_stamped.transform.translation.x = msg.pose.pose.position.x
        transform_stamped.transform.translation.y = msg.pose.pose.position.y
        transform_stamped.transform.translation.z = msg.pose.pose.position.z

        # Extract orientation
        transform_stamped.transform.rotation.x = msg.pose.pose.orientation.x
        transform_stamped.transform.rotation.y = msg.pose.pose.orientation.y
        transform_stamped.transform.rotation.z = msg.pose.pose.orientation.z
        transform_stamped.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    odom_to_tf_node = OdomToTfNode()
    rclpy.spin(odom_to_tf_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
