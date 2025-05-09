#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose


class Init(Node):

    def __init__(self):
        super().__init__('pid')

        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.objetivo_sub = self.create_subscription(Pose, 'pose_objetivo', self.pose_objetivo_callback, 10)

        self.pose_pub = self.create_publisher(Pose, 'output', 10)
        self.init_pub = self.create_publisher(Int32, 'iniciador', 10)

        self.kp_ = [0]*3
        self.ki_ = [0]*3
        self.kd_ = [0]*3

        self.pose_actual_ = [0]*3
        self.pose_objetivo_ = [0]*3
        self.pose_output_ = [0]*3

        self.error_anterior_ = [0]*3
        self.integral_ = [0]*3

        self.kp_[0] = 0.9
        self.kp_[1] = 1.0
        self.kp_[2] = 1.0

        self.ki_[0] = 0.1
        self.ki_[1] = 0.1
        self.ki_[2] = 0

        self.kd_[0] = 0.05
        self.kd_[1] = 0.05
        self.kd_[2] = 0.05

        # iniciador = float()
        # iniciador.data = 1.0

        # self.init_pub.publish(iniciador)

        self.tiempo_anterior_ = self.get_clock().now().nanoseconds
        self.create_timer(0.1, self.calcularpid)

    def pose_callback(self, msg):
        self.pose_actual_[0] = msg.position.x
        self.pose_actual_[1] = msg.position.y
        self.pose_actual_[2] = msg.position.z

    def pose_objetivo_callback(self, msg):
        self.pose_objetivo_[0] = msg.position.x
        self.pose_objetivo_[1] = msg.position.y
        self.pose_objetivo_[2] = msg.position.z


    def calcularpid(self):
        tiempo_actual = self.get_clock().now().nanoseconds
        dt = (tiempo_actual - self.tiempo_anterior_) / 1e9

        for n in range(3):
            error = self.pose_objetivo_[n] - self.pose_actual_[n]

            self.integral_[n] += error * dt
            derivada = (error - self.error_anterior_[n]) / dt #if dt > 0 else 0

            p = self.kp_[n] * error
            i = self.ki_[n] * self.integral_[n]
            d = self.kd_[n] * derivada

            self.pose_output_[n] = p + i + d
            self.error_anterior_[n] = error

        self.tiempo_anterior_ = tiempo_actual


        control = Pose()

        control.position.x = self.pose_actual_[0] + self.pose_output_[0] * dt
        control.position.y = self.pose_actual_[1] + self.pose_output_[1] * dt
        control.position.z = self.pose_actual_[2] + self.pose_output_[2] * dt

        self.pose_pub.publish(control)


def main(args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()