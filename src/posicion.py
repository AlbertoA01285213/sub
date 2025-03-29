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
        
        self.iniciador_data = Int32()

    def pose_output_callback(self, msg):
        pose = Pose()
        if self.iniciador_data.data == 1:
            pose.position.x = msg.position.x
            pose.position.y = msg.position.y
            pose.position.z = msg.position.z
        else:
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0

        self.pose_pub.publish(pose)

    def iniciador_callback(self, msg):
        self.iniciador_data = msg

def main(args=None):
    rclpy.init(args=args)

    node = Init()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()