#!/usr/bin/env python3

"""
Este nodo debe publicar las coordenadas por un topico para que el nodo de comunicacion, se las envie al arduino
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread, Event
from time import sleep

class ArduinoCoordPubNode(Node):
    def __init__(self):
        super().__init__('arduino_coord_pub_node')

        self.get_logger().info('[Arduino Coord Pub Node]: ha sido iniciado')

        # Publicadores y Subscriptores

        self.pub = self.create_publisher(String, 'arduino/command/coord', 10)
        self.get_coord()
    
    
    def get_coord(self):
        """
        De momento pide un input para mandar por consola una coordenada
        """ 

        while True:
            msg = str(input("Escribe un mensaje"))
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    arduino_coord_pub = ArduinoCoordPubNode()
    rclpy.spin(arduino_coord_pub)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
