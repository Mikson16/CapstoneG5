#!/usr/bin/env python3

"""
Este nodo debera levantar la comunicacion serial con el Arduino y solo mandar mensajes ya procesados.
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
        self.subscription = self.create_subscription(String, 'arduino/command/coord', self.coord_command_callback, 10) # esta suscripcion debe recibir un mensaje de las coordenadas a traves del topico 
        
        self.emergency_sub = self.create_subscription(String, 'arduino/command/emergency', self.emergency_callback, 10)
        # Atributo de mensaje
        self.msg = None
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

        
        # self.serial_pub_thread = Thread(target=self.send_serial_msg, name = 'serial_pub_thread') # de momento no sera daemon
        # self.serial_pub_thread.start()
        # Usando un timer
        self.timer = self.create_timer(0.1, self.send_serial_msg)   
        self.send_serial_msg()

    def emergency_callback(self, msg):
        try:
            data = str(msg.data) + '\n'
            self.ser.write(data.encode('utf-8'))
            self.get_logger().info(f'Enviando senal de paro de emergencia')
        except Exception as e:
            self.get_logger().warning(f'Error al mandar mensaje de emergencia por serial')
        

    def send_serial_msg(self):

        if self.msg is None:
            return
        try:
            self.get_logger().info('Mandar mensaje serial')
            data = str(self.msg) + '\n'
            self.ser.write(data.encode('utf-8'))
            self.get_logger().info(f'SE ESTA ENVIANDO EL MENSAJE {data}')
            self.msg = None # Para resetear y evitar que publique el mismo mensaje varias veces
        except Exception as e:
            self.get_logger().error(f'[ARDUINO-COM]: Error escribiendo al serial: {e}')


    def coord_command_callback(self, msg):
        self.get_logger().info(f'Coordenadas recibidas: {msg.data}, mandando por serial')
        self.msg = msg.data

    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('[ARDUINO-COM]: Deteniendo hilos de procesamiento')
        # Señalar al hilo que debe detenerse
        self._stop_event.set()
        # Esperar a que termine el hilo de publicación serial si existe
        try:
            if hasattr(self, 'serial_pub_thread') and self.serial_pub_thread.is_alive():
                self.serial_pub_thread.join(timeout=2.0)
            self.timer.cancel()
        except Exception as e:
            self.get_logger().error(f'[ARDUINO-COM]: Error al unir hilo: {e}')
        self.get_logger().info('[ARDUINO-COM]: Hilos detenidos')

def main(args=None):
    rclpy.init(args=args)
    arduino_com_node = ArduinoComNode(port='/dev/ttyUSB0')# En caso de testear ocn arduino uno es /dev/ttyAMA0, con arduino mega /dev/ttyACM0
    # /dev/ttyUSB0 Para arduino generico, hayq ue darle permisos
    # arduino_com_node.port('/dev/ttyACM0')
    rclpy.spin(arduino_com_node)
    arduino_com_node.destroy_threads()
    arduino_com_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
