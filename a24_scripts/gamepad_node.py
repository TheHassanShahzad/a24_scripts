import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from approxeng.input.selectbinder import ControllerResource
import math

class GamepadToOdomNode(Node):
    def __init__(self):
        super().__init__('gamepad_to_odom_node')
        self.publisher_ = self.create_publisher(Twist, '/odom', 10)
        self.timer = self.create_timer(0.001, self.timer_callback)
        
        self.max_linear_vel = 0.22  # m/s
        self.max_angular_vel = 0.5  # rad/s

        self.get_logger().info('Gamepad to Odom Node has been started.')

    def timer_callback(self):
        twist = Twist()
        while True:
            try:
                with ControllerResource() as joystick:
                    while joystick.connected:
                        right_stick_y = joystick['ry']
                        right_stick_x = joystick['rx']
                        print(right_stick_x, right_stick_y)

                        twist.linear.x = -right_stick_y * self.max_linear_vel
                        twist.angular.z = right_stick_x * self.max_angular_vel

                        self.publisher_.publish(twist)
                    # self.get_logger().info(f'Publishing: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}')
            
            except IOError:
                self.get_logger().error('No controller found.')

def main(args=None):
    rclpy.init(args=args)
    node = GamepadToOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
