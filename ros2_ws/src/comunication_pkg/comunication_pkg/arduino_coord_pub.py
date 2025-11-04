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


        self.input_thread =  Thread(target=self.get_coord, daemon=True)
        self.input_thread.start()
    
    
    def get_coord(self):
        """
        De momento pide un input para mandar por consola una coordenada
        """ 

        while rclpy.ok():
            msg = str(input("Escribe una coordenada: "))
            self.pub.publish(String(data=msg))

            #!TODO Nota para el yo de mañana que cansado se olvidara de lo que hizo
            """
            Recibir los mensajes por el input de consola es un cacho, seria mejor que para una demostracion, hacer una rutina de coordenadas en una lista y recorrerla mientras se envian, ambos nodos deberian funcionar a traves de un run nodo y no con el launch
            """


def main(args=None):
    rclpy.init(args=args)
    arduino_coord_pub = ArduinoCoordPubNode()
    rclpy.spin(arduino_coord_pub)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
