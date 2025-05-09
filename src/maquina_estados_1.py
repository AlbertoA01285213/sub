#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float64MultiArray
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
from math import sqrt

class Init(Node):
    def __init__(self):
        super().__init__('maquina_estados_1')

        self.pose_sub = self.create_subscription(Pose, 'uuv/state/pose', self.pose_callback, 10)
        self.objetivo_sub = self.create_subscription(Pose, 'pose_objetivo', self.objetivo_callback, 10)

        self.estado = self.create_publisher(Int32, 'estado', 10)

        self.pose = [0.0]*6
        self.objetivo = [0.0]*6

        self.estado_num = Int32()
        self.estado_num.data = 0
        # self.estado_alcanzado = False

        self.error = [0]*6

        self.timer = self.create_timer(0.1, self.calc_estado)

    def pose_callback(self,msg):
        self.pose[0] = msg.position.x
        self.pose[1] = msg.position.y

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.pose[3] = roll
        self.pose[4] = pitch
        self.pose[5] = yaw

    def objetivo_callback(self,msg):
        self.objetivo[0] = msg.position.x
        self.objetivo[1] = msg.position.y
        self.objetivo[2] = msg.position.z

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.objetivo[3] = roll
        self.objetivo[4] = pitch
        self.objetivo[5] = yaw

    def calc_estado(self):
        for n in range(6):
            self.error[n] = self.objetivo[n] - self.pose[n]

        error_total = self.error[0] + self.error[1] + self.error[2] + self.error[3] + self.error[4] + self.error[5]

        if abs(error_total) < 0.06:
            self.estado_num.data += 1

        self.estado.publish(self.estado_num)

    # def calc_estado(self):
    #     error_lineal = sqrt(sum((self.objetivo[i] - self.pose[i])**2 for i in range(3)))
    #     error_angular = sqrt(sum((self.objetivo[i+3] - self.pose[i+3])**2 for i in range(3)))

    #     umbral_lineal = 10   # 5 cm
    #     umbral_angular = 10    # ~2.8 grados

    #     if error_lineal < umbral_lineal and error_angular < umbral_angular:
    #         if not self.estado_alcanzado:
    #             self.estado_num.data += 1
    #             self.get_logger().info(f'Nuevo estado alcanzado: {self.estado_num.data}')
    #             self.estado_alcanzado = True
    #     else:
    #         self.estado_alcanzado = False

    #     self.estado.publish(self.estado_num)


def main(args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()