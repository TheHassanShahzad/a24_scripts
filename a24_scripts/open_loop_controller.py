import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

class CmdVelToPWMNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_pwm')
        
        # Subscriber to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher to /pwm_data topic
        self.publisher = self.create_publisher(
            Int16MultiArray,
            '/pwm_data',
            10)
        
        self.wheel_spacing = 0.226
        self.max_motor_linear_vel = 0.22

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.pwm_right = 0
        self.pwm_left = 0

        self.right_wheel_vel = 0.0
        self.left_wheel_vel = 0.0

        self.processed_right_wheel_vel = 0.0
        self.processed_left_wheel_vel = 0.0

    def cmd_vel_callback(self, msg):
        # Print received Twist message
        # self.get_logger().info(f'Received cmd_vel - Linear: {msg.linear.x}, Angular: {msg.angular.z}')
        
        # Store received values
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        self.get_wheel_vels(self.linear_x, self.angular_z, self.wheel_spacing)

        if (self.right_wheel_vel > self.max_motor_linear_vel or self.left_wheel_vel > self.max_motor_linear_vel):
            if self.right_wheel_vel >= self.right_wheel_vel:
                scale_factor = self.max_motor_linear_vel/self.right_wheel_vel
            else:
                scale_factor = self.max_motor_linear_vel/self.left_wheel_vel

            new_linear_x = scale_factor*self.linear_x
            new_angular_z = scale_factor*self.angular_z

            self.get_wheel_vels(new_linear_x, new_angular_z, self.wheel_spacing)
            
        print(self.right_wheel_vel, self.left_wheel_vel)
        self.get_PWM(self.right_wheel_vel, self.left_wheel_vel, self.max_motor_linear_vel)

        # Publish PWM values
        pwm_data = Int16MultiArray()
        pwm_values = [self.pwm_right, self.pwm_left]
        pwm_data.data = pwm_values
        self.publisher.publish(pwm_data)
        # self.get_logger().info(f'Published PWM values: {pwm_values}')


    def get_wheel_vels(self, lin_x, ang_z, wheel_spacing):
        self.right_wheel_vel, self.left_wheel_vel = lin_x + ((ang_z*wheel_spacing)/2), lin_x - ((ang_z*wheel_spacing)/2)   

    def get_PWM(self, right_vel, left_vel, max_motor_vel):
        if right_vel >= 0:
            self.pwm_right = int((right_vel / max_motor_vel) * 255)
        else:
            self.pwm_right = -int((abs(right_vel) / max_motor_vel) * 255)
        
        # Clamp the right PWM value between -255 and 255
        self.pwm_right = max(min(self.pwm_right, 255), -255)

        if left_vel >= 0:
            self.pwm_left = int((left_vel / max_motor_vel) * 255)
        else:
            self.pwm_left = -int((abs(left_vel) / max_motor_vel) * 255)

        # Clamp the left PWM value between -255 and 255
        self.pwm_left = max(min(self.pwm_left, 255), -255)



  

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPWMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
