#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray

import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('Publicador')

        # Configura tu puerto serial y baudrate aquí
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1) #/dev/ttyACM0  /dev/ttyUSB0
            self.get_logger().info('Puerto serial abierto correctamente')
        except serial.SerialException as e:
            self.get_logger().error(f'Error al abrir el puerto serial: {e}')
            return
        
        self.motores = self.create_subscription(Float64MultiArray, 'uuv/forces', self.input,10)

        self.publisher_ = self.create_publisher(String, 'serial_data', 10)

        self.entrada = [0.0]*6

        self.timer = self.create_timer(0.1, self.write_serial)

    def input(self, msg):
        self.entrada[0] = msg.data[0]
        self.entrada[1] = msg.data[1]
        self.entrada[2] = msg.data[2]
        self.entrada[3] = msg.data[3]
        self.entrada[4] = msg.data[4]
        self.entrada[5] = msg.data[5]

    def write_serial(self):
        # try:
        #     data = String(self.entrada[0])+ '\n'
        #     self.serial_port.write(data.encode('utf-8'))
        #     self.get_logger().info(f'Dato enviado: {msg.data}')
        # except Exception as e:
        #     self.get_logger().error(f'Error al escribir en el puerto serial: {e}')
        try:
            # Convertir lista de floats a string tipo CSV
            data_str = ','.join([f'{x:.2f}' for x in self.entrada]) + '\n'
            self.serial_port.write(data_str.encode('utf-8'))

            # Publicar en ROS también (opcional)
            ros_msg = String()
            ros_msg.data = data_str.strip()
            self.publisher_.publish(ros_msg)

            self.get_logger().info(f'Dato enviado al serial: {ros_msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error al escribir en el puerto serial: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
