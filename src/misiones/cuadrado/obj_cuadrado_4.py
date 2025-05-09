#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float64MultiArray
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
from math import radians

class Init(Node):
    def __init__(self):
        super().__init__('objetivo_2')

        self.estado_sub = self.create_subscription(Int32, 'estado', self.estado_callback, 10)

        self.objetivo_pub = self.create_publisher(Pose, 'pose_objetivo', 10)

        self.posicion_objetivo = Pose()
        self.estado_num = 0

        self.timer = self.create_timer(0.1, self.mision)

    def estado_callback(self, msg):
        self.estado_num = msg.data


    def mision(self):
        if self.estado_num == 3:
            self.publish_pose()

        else:
            pass

    def publish_pose(self):
        roll = radians(0.0)
        pitch = radians(0.0)
        yaw = radians(180.0)

        x,y,z,w = quaternion_from_euler(roll, pitch, yaw)

        self.posicion_objetivo.position.x = 0.0
        self.posicion_objetivo.position.y = 2.0
        self.posicion_objetivo.position.z = -1.0
        self.posicion_objetivo.orientation.x = x
        self.posicion_objetivo.orientation.y = y
        self.posicion_objetivo.orientation.z = z
        self.posicion_objetivo.orientation.w = w
        
        self.objetivo_pub.publish(self.posicion_objetivo)

def main(args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()