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

        self.kp_= [0]*3
        self.ki_ = [0]*3
        self.kd_ = [0]*3

        self.pose_actual_ = [0]*3
        self.pose_objetivo_ = [0]*3
        self.pose_output_ = [0]*3

        self.error_anterior_ = [0]*3
        self.integral_ = [0]*3

        self.kp_[0] = 0.01
        self.kp_[1] = 0.01
        self.kp_[2] = 0.01
        # self.kp_[3] = 1
        # self.kp_[4] = 1
        # self.kp_[5] = 1

        self.ki_[0] = 0
        self.ki_[1] = 0
        self.ki_[2] = 0
        # self.ki_[3] = 0
        # self.ki_[4] = 0
        # self.ki_[5] = 0

        self.kd_[0] = 0
        self.kd_[1] = 0
        self.kd_[2] = 0
        # self.kd_[3] = 0
        # self.kd_[4] = 0
        # self.kd_[5] = 0

        iniciador = Int32()
        iniciador.data = 1

        self.init_pub.publish(iniciador)

        self.tiempo_anterior_ = self.get_clock().now().nanoseconds
        self.create_timer(0.1, self.calcularpid)

    def pose_callback(self, msg):
        self.pose_actual_[0] = msg.position.x
        self.pose_actual_[1] = msg.position.y
        self.pose_actual_[2] = msg.position.z

        # q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # roll, pitch, yaw = tf.euler_from_quaternion(q)

        # self.pose_actual_[3] = roll
        # self.pose_actual_[4] = pitch
        # self.pose_actual_[5] = yaw

    def pose_objetivo_callback(self, msg):
        self.pose_objetivo_[0] = msg.position.x
        self.pose_objetivo_[1] = msg.position.y
        self.pose_objetivo_[2] = msg.position.z

        # q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # roll, pitch, yaw = tf.euler_from_quaternion(q)

        # self.pose_objetivo_[3] = roll
        # self.pose_objetivo_[4] = pitch
        # self.pose_objetivo_[5] = yaw

    def calcularpid(self):
        tiempo_actual = self.get_clock().now().nanoseconds
        dt = (tiempo_actual - self.tiempo_anterior_) / 1e9

        for n in range(3):
            error = self.pose_objetivo_[n] - self.pose_actual_[n]

            self.integral_[n] += error * dt
            derivada = (error - self.error_anterior_[n]) / dt if dt > 0 else 0

            p = error * self.kp_[n]
            i = self.integral_[n] * self.ki_[n]
            d = derivada * self.kd_[n]

            self.pose_output_[n] = p + i + d
            self.error_anterior_[n] = error

        self.tiempo_anterior_ = tiempo_actual

        msg = Pose()
        msg.position.x = self.pose_output_[0]
        msg.position.y = self.pose_output_[1]
        msg.position.z = self.pose_output_[2]

        #q = tf.quaternion_from_euler(self.pose_output_[3], self.pose_output_[4], self.pose_output_[5])
        #msg.orientation.x = q[0]
        #msg.orientation.y = q[1]
        #msg.orientation.z = q[2]
        #msg.orientation.w = q[3]

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()