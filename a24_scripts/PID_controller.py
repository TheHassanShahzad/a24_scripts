#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
import numpy as np

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.wheel_spacing = 0.226
        self.wheel_diameter = 0.07

        # Initialize variables
        self.current_twist_linear_x = 0.0
        self.current_twist_angular_z = 0.0
        self.desired_twist_linear_x = 0.0
        self.desired_twist_angular_z = 0.0

        self.right_wheel_error = 0.0
        self.left_wheel_error = 0.0

        # PID gains (adjust as needed)
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.pwm_scaler = 1500

        # Integrator and differentiator terms for PID
        self.right_wheel_integral = 0.0
        self.left_wheel_integral = 0.0
        self.right_wheel_prev_error = 0.0
        self.left_wheel_prev_error = 0.0

        # PWM limits
        self.min_pwm = -255
        self.max_pwm = 255

        # Publisher for PWM data
        self.pwm_pub = self.create_publisher(Int16MultiArray, '/pwm_data', 10)

        # Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Subscriber to /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

    def cmd_vel_callback(self, msg):
        self.desired_twist_linear_x = msg.linear.x
        self.desired_twist_angular_z = msg.angular.z
        self.calculate_pwm()

    def odom_callback(self, msg):
        self.current_twist_linear_x = msg.twist.twist.linear.x
        self.current_twist_angular_z = msg.twist.twist.angular.z
        self.calculate_pwm()

    def calculate_pwm(self):
        self.get_wheel_vel_error(
            self.current_twist_linear_x,
            self.current_twist_angular_z,
            self.desired_twist_linear_x,
            self.desired_twist_angular_z
        )

        right_wheel_pwm = self.pid_control(self.right_wheel_error, self.right_wheel_integral, self.right_wheel_prev_error)
        left_wheel_pwm = self.pid_control(self.left_wheel_error, self.left_wheel_integral, self.left_wheel_prev_error)

        # print(right_wheel_pwm, left_wheel_pwm)
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [int(right_wheel_pwm), int(left_wheel_pwm)]
        self.pwm_pub.publish(pwm_msg)

    def get_wheel_vel_error(self, cur_lin_x, cur_ang_z, des_lin_x, des_ang_z):
        lin_x_error = des_lin_x - cur_lin_x
        ang_z_error = des_ang_z - cur_ang_z

        self.right_wheel_error = lin_x_error + ((self.wheel_spacing * ang_z_error) / 2)
        self.left_wheel_error = lin_x_error - ((self.wheel_spacing * ang_z_error) / 2)

    def pid_control(self, error, integral, prev_error):
        # PID calculations
        P = self.Kp * error
        I = self.Ki * (integral + error)
        D = self.Kd * (error - prev_error)

        # Update integral and prev_error
        integral += error
        prev_error = error

        # Compute output
        output = (P + I + D)*self.pwm_scaler

        # Clamp to PWM limits
        output = max(self.min_pwm, min(self.max_pwm, output))

        return output

def main(args=None):
    rclpy.init(args=args)
    pid_controller_node = PIDControllerNode()
    rclpy.spin(pid_controller_node)
    pid_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
