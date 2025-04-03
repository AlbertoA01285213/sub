#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose

class Init(Node):

    def __init__(self):
        super().__init__('trayectoria')

        self.objetivo = self.create_subscription(Pose, '', self.pose_objetivo_callback, 10)
        self.posicion = self.create_subscription(Pose, '', self.pose_posicion_callback, 10)

        self.pose_pub = self.create_publisher(Pose, 'pose', 10) #cambiar publicador

        self.create_timer(0.1, self.calculartrayectoria)

        
    def pose_objetivo_callback(self, msg):
        pose_objetivo = Pose()

        pose_objetivo.position.x = msg.position.x
        pose_objetivo.position.y = msg.position.y
        pose_objetivo.position.z = msg.position.z

    def pose_posicion_callback(self, msg):
        pose_posicion = Pose()

        pose_posicion.position.x = msg.position.x
        pose_posicion.position.y = msg.position.y
        pose_posicion.position.z = msg.position.z

    def calculartrayectoria(self):
        for i in range(2):
            m

    


def main    (args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()