#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float64MultiArray
from geometry_msgs.msg import Pose


class Init(Node):

    def __init__(self):
        super().__init__('pid')

        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.objetivo_sub = self.create_subscription(Pose, 'pose_objetivo', self.pose_objetivo_callback, 10)

        self.thruster_pub = self.create_publisher(Float64MultiArray, 'uuv/forces', 10)
        
        self.kp_ = [0]*6
        self.ki_ = [0]*6
        self.kd_ = [0]*6

        self.pose_actual_ = [0]*6
        self.pose_objetivo_ = [0]*6
        self.pose_output_ = [0]*6

        self.error_anterior_ = [0]*6
        self.integral_ = [0]*6

        self.kp_[0] = 100.0
        self.kp_[1] = 50.0
        self.kp_[2] = 100.0
        self.kp_[3] = 5.0
        self.kp_[4] = 5.0
        self.kp_[5] = 5.0

        self.ki_[0] = 10.0
        self.ki_[1] = 10.0
        self.ki_[2] = 10.0
        self.ki_[3] = 1.0
        self.ki_[4] = 1.0
        self.ki_[5] = 1.0

        self.kd_[0] = 300.0
        self.kd_[1] = 300.0
        self.kd_[2] = 300.0
        self.kd_[3] = 10.0
        self.kd_[4] = 10.0
        self.kd_[5] = 10.0

        self.tiempo_anterior_ = self.get_clock().now().nanoseconds
        self.create_timer(0.1, self.calcularpid)

    def pose_callback(self, msg):
        self.pose_actual_[0] = msg.position.x
        self.pose_actual_[1] = msg.position.y
        self.pose_actual_[2] = msg.position.z
        self.pose_actual_[3] = msg.orientation.x
        self.pose_actual_[4] = msg.orientation.y
        self.pose_actual_[5] = msg.orientation.z

    def pose_objetivo_callback(self, msg):
        self.pose_objetivo_[0] = msg.position.x
        self.pose_objetivo_[1] = msg.position.y
        self.pose_objetivo_[2] = msg.position.z
        self.pose_objetivo_[3] = msg.orientation.x
        self.pose_objetivo_[4] = msg.orientation.y
        self.pose_objetivo_[5] = msg.orientation.z


    def calcularpid(self):
        tiempo_actual = self.get_clock().now().nanoseconds
        dt = (tiempo_actual - self.tiempo_anterior_) / 1e9

        for n in range(6):
            error = self.pose_objetivo_[n] - self.pose_actual_[n]

            self.integral_[n] += error * dt
            derivada = (error - self.error_anterior_[n]) / dt #if dt > 0 else 0

            p = self.kp_[n] * error
            i = self.ki_[n] * self.integral_[n]
            d = self.kd_[n] * derivada

            self.pose_output_[n] = p + i + d
            self.error_anterior_[n] = error

        self.tiempo_anterior_ = tiempo_actual


        control = Float64MultiArray()
        control.data = [0.0]*6

        # control.position.x = self.pose_actual_[0] + self.pose_output_[0] * dt
        # control.position.y = self.pose_actual_[1] + self.pose_output_[1] * dt
        # control.position.z = self.pose_actual_[2] + self.pose_output_[2] * dt

        control.data[0] = float(self.pose_output_[0])
        control.data[1] = float(self.pose_output_[1])
        control.data[2] = float(self.pose_output_[2])
        control.data[3] = float(self.pose_output_[3])
        control.data[4] = float(self.pose_output_[4])
        control.data[5] = float(self.pose_output_[5])

        self.thruster_pub.publish(control)


def main(args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()