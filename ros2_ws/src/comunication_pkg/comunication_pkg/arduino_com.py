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
        self.create_subscription(String, 'arduino/command/coord', self.coord_command_callback, 10) # esta suscripcion debe recibir un mensaje de las coordenadas a traves del topico 

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

        
        self.serial_pub_thread = Thread(target=self.send_serial_msg, name = 'serial_pub_thread') # de momento no sera daemon
        self.serial_pub_thread.start()
        # Usando un timer
        # self.timer = self.create_timer(1.0, self.send_serial_msg)   
        # self.send_serial_msg()


    def send_serial_msg(self, msg_data):
        # pedir el mensaje por consola
        while not self._stop_event.is_set():
            try:
                self.get_logger().info('Mandar mensaje serial')
                # msg = str(input(f'Escribe un mensaje para mandar al arduino: ')) +'\n'

                try: 
                    self.ser.write(msg_data.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f'[ARDUINO-COM]: Error escribiendo al serial: {e}')
                
            except Exception as e:
                self.get_logger().error(f'ARDUINO-COM: Error en el loop de consola: {e}')


    def coord_command_callback(self, msg):
        self.get_logger().info(f'Coordenadas recibidas: {msg.data}, mandando por serial')
        self.send_serial_msg(msg.data)

    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('[ARDUINO-COM]: Deteniendo hilos de procesamiento')
        self.stop_event.set()
        self.processing_thread.join()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        self.get_logger().info('[ARDUINO-COM]: Hilos detenidos')

def main(args=None):
    rclpy.init(args=args)
    arduino_com_node = ArduinoComNode(port='/dev/ttyACM0')
    # arduino_com_node.port('/dev/ttyACM0')
    rclpy.spin(arduino_com_node)
    arduino_com_node.destroy_threads()
    arduino_com_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
