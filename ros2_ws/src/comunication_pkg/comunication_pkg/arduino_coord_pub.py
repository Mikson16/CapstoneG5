#!/usr/bin/env python3

"""
Este nodo debe colectar la informacion que venga de los demas nodos y mandarsela al nodo de comunicacion serial
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from threading import Thread, Event
from time import sleep

class ArduinoCoordPubNode(Node):
    def __init__(self):
        super().__init__('arduino_coord_pub_node')

        self.get_logger().info('[Arduino Coord Pub Node]: ha sido iniciado')


        # Lista de mensajes de prueba
        self.test_msg_serial_list = ['0', '1', 'holaaa', '2324', '67', '1726861387', '333333', '29387928', 'aaaaa', 'kjcbhawkuckbshdc', '1234567890987654321', 'qwertyuioolkjhgfdsazxcvbn']
        # Publicadores y Subscriptores

        self.pub = self.create_publisher(String, 'arduino/command/coord', 10)

        self.kinematics_sub = self.create_subscription(Int16MultiArray, 'inv_kinematics/angles', self.kinematics_callback, 10) # mensaje de la cinematica

        self.emergency_sub = self.create_subscription(Int16MultiArray, 'emergency/msg', self.emergency_callback, 10) # mensaje del paro de emergencia de software

        self.orientation_sub = self.create_subscription(Int16MultiArray, )


        self.input_thread =  Thread(target=self.get_coord, daemon=True)
        self.input_thread.start()
    
    
    def get_coord(self):
        """
        De momento pide un input para mandar por consola una coordenada
        """ 
        # Contador para testeo de mensaje
        ct = 0
        while rclpy.ok():
            msg = String()
            msg.data = self.test_msg_serial_list[ct]
            self.pub.publish(msg)

            ct +=1
            if ct >= len(self.test_msg_serial_list):
                ct = 0

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
