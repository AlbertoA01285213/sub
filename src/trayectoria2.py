#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose

class Init(Node):

    def __init__(self):
        super().__init__('trayectoria2')

        self.objetivo = self.create_subscription(Pose, '', self.pose_objetivo_callback, 10) # Pose, para saber por donde va el sub
        self.posicion = self.create_subscription(Pose, '', self.pose_posicion_callback, 10)

        self.pose_pub = self.create_publisher(Pose, 'pose', 10)                             # Aceleraciones

                #   x  y  z
        self.pi_ = [0, 0, 0]    # Crear punto inicial
        self.pf_ = [10, 10, 10]    # Crear punto final

        self.ti = 0.0           # Crear tiempo inicial
        self.tf = 10.0          # Crear tiempo final

        self.p = 100             # La cantidad de puntos a crear

        self.vector_dir_ = [0]*3    # Crea un vector de direccion
        self.vector_uni_ = [0]*3    # Crea el vector unitario de la direccion

    

    def crear_wp(self):
        distancia = np.sqrt((self.pf_[0]-self.pi_[0])^2 + (self.pf_[1]-self.pi_[1])^2 + (self.pf_[2]-self.i_[2])^2)

        self.vector_dir_[0] = self.pf_[0] - self.pi_[0] 
        self.vector_dir_[1] = self.pf_[1] - self.pi_[1] 
        self.vector_dir_[2] = self.pf_[2] - self.pi_[2] 

        self.vector_uni_[0] = self.vector_dir_[0] / distancia
        self.vector_uni_[1] = self.vector_dir_[1] / distancia
        self.vector_uni_[2] = self.vector_dir_[2] / distancia





