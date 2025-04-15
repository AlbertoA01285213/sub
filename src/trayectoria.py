#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose

class Init(Node):

    def __init__(self):
        super().__init__('trayectoria')

        self.objetivo = self.create_subscription(Pose, '', self.pose_objetivo_callback, 10)
        self.posicion = self.create_subscription(Pose, '', self.pose_posicion_callback, 10)

        self.pose_pub = self.create_publisher(Pose, 'pose', 10) #cambiar publicador

        self.d = np.array([[0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0]])
        
        self.ti = 0.0
        self.tf = 10.0
        self.presicion = 10

        self.create_timer(0.1, self.calculartrayectoria)

        
    def pose_objetivo_callback(self, msg):
        self.pose_objetivo = Pose()

        self.pose_objetivo.position.x = msg.position.x
        self.pose_objetivo.position.y = msg.position.y
        self.pose_objetivo.position.z = msg.position.z

    def pose_posicion_callback(self, msg):
        self.pose_posicion = Pose()

        self.pose_posicion.position.x = msg.position.x
        self.pose_posicion.position.y = msg.position.y
        self.pose_posicion.position.z = msg.position.z

    def calculartrayectoria(self):
        self.d = np.array([[self.pose_posicion.position.x, 0, 0, self.pose_objetivo.position.x, 0, 0, self.ti, self.tf],
                            [self.pose_posicion.position.y, 0, 0, self.pose_objetivo.position.y, 0, 0, self.ti, self.tf],
                            [self.pose_posicion.position.z, 0, 0, self.pose_objetivo.position.z, 0, 0, self.ti, self.tf]])

        t = np.linspace(self.ti, self.tf, self.presicion)
        c = np.ones((t, t))
        m = np.array([[1, self.ti, (self.ti)^2, (self.ti)^3, (self.ti)^4, (self.ti)^5],
                      [0, 1, 2*self.ti, 3*(self.ti)^2, 4*(self.ti)^3, 5*(self.ti)^4],
                      [0, 0, 2, 6*self.ti, 12*(self.ti)^2, 20*(self.ti)^3],
                      [1, self.tf, (self.tf)^2, (self.tf)^3, (self.tf)^4, (self.tf)^5],
                      [0, 1, 2*self.tf, 3*(self.tf)^2, 4*(self.tf)^3, 5*(self.tf)^4],
                      [0, 0, 2, 6*self.tf, 12*(self.tf)^2, 20*(self.tf)^3]])

        a = np.linalg.inv(m)*np.transpose(self.d)


        for i in range(3):


            

    


def main    (args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()