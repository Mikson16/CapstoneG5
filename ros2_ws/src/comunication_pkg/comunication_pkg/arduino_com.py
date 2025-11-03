#!/usr/bin/env python3

"""
Este nodo debera levantar la comunicacion serial con el Arduino.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread, Event
import serial
import time


class ArduinoComNode(Node):

    def __init__(self, port=None, baudrate = 115200, timeout=1.0):
        super().__init__('arduino_com_node')

    # agregar subscriptores y publicadores cuando sea necesario

        self.baudrate = baudrate
        self.timeout = timeout

        self.port = port 
 

        if self.port == None:
            self.get_logger().fatal('No se determino puerto serial, Especifica port=/dev/ttyUSB0')
            raise RuntimeError('No se encontro puerto serial')
        
        self.get_logger().info(f'Abriendo puerto serial {self.port} @ {self.baudrate}')
        try:
            self.ser = serial.Serial(self.port, self.baudrate)
            self.get_logger().info(f"[ARDUINO-COM]: Creacion elemento serial")

            # tiempo para darle a arduino
            time.sleep(2.0)
        except Exception as e:
            self.get_logger().fatal(f'Error abriendo la comunicacion serial: {e}')

        self._stop_event = Event()

        # Usando un timer
        self.timer = self.create_timer(1.0, self.send_serial_msg)   
        # self.send_serial_msg()


    def send_serial_msg(self):
        # pedir el mensaje por consola
        msg = str(input(f'Escribe un mensaje para mandar al arduino: ')) +'\n'

        try: 
            self.ser.write(msg.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'[ARDUINO-COM]: Error escribiendo al serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    arduino_com_node = ArduinoComNode(port='/dev/ttyACM0')
    # arduino_com_node.port('/dev/ttyACM0')
    rclpy.spin(arduino_com_node)
    arduino_com_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
