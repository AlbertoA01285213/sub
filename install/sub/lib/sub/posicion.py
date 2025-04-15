#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose



class Init(Node):

    def __init__(self):
        super().__init__('posicion')

        self.pose_output_sub = self.create_subscription(Pose, 'output', self.pose_output_callback, 10)
        self.iniciador_sub = self.create_subscription(Int32, 'iniciador', self.iniciador_callback, 10)

        self.pose_pub = self.create_publisher(Pose, 'pose', 10)
        self.inicio = 1.0

        self.posicion = Pose()

        self.create_timer(0.1, self.posicionamiento)

    def pose_output_callback(self, msg):
        self.posicion.position.x = msg.position.x
        self.posicion.position.y = msg.position.y
        self.posicion.position.z = msg.position.z


    def iniciador_callback(self, msg):
        self.inicio = msg.data

    def posicionamiento(self):
        self.pose_pub.publish(self.posicion)
        

        # if self.inicio == 1.0:
        #     self.pose_pub.publish(self.posicion)


        # elif self.inicio == 0.0:
        #     self.posicion.position.x = 0.0
        #     self.posicion.position.y = 0.0
        #     self.posicion.position.z = 0.0
        #     self.pose_pub.publish(self.posicion)

        # else:
        #     self.posicion.position.x = 2.0
        #     self.posicion.position.y = 2.0
        #     self.posicion.position.z = 2.0
        #     self.pose_pub.publish(self.posicion)


def main    (args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()