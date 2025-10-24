#!/usr/bin/env python3

"""
Nodo que recibe la imagen de la camara estatica y la procesa para identificar donde se encuentra la bolsa de papa en la imagen.

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from threading import Thread
from time import sleep
import numpy as np
from threading import Thread, Event
from queue import Queue, Empty

class StaticCameraPapaNode(Node):
    def __init__(self):
        super().__init__('static_camera_papa_node')

        self.get_logger().info('[Static Camera Papa Node]: ha sido iniciado')

        # Crear cola para pasar imagenes del callback al hilo de procesamiento
        self.img_q = Queue(maxsize=5) # Cola Fifo de tamaño 5
        self.stop_event = Event()

        # Crear suscriptor al mensaje de la camara estatica
        self.subscription = self.create_subscription(Image, {'static_camera/image_raw'}, self.queue_papa_callback, 10)
        self.get_logger().info('[Static Camera Papa Node]: Suscriptor creado')

        # Bridge
        self.bridge = CvBridge()

        # Iniciar hilo de procesamiento
        self.processing_thread = Thread(target=self.processing_loop)
        self.processing_thread.start() # inicio el Hilo, no uso un hilo daemon, porque Ros2 ya lo hace internamente
        self.get_logger().info('[Static Camera Papa Node]: Hilo de procesamiento iniciado')

    def queue_papa_callback(self, msg):
        """
        Esta callback recibe el mensaje imagen y debe procesarlo a formato cv2 y ponerlo en la cola si es posible
        """
        try:
            cv2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[Static Camera Papa Node]: Error al convertir la imagen: {e}')
            return
        
        # Ingresar a la cola
        try:
            self.img_q.put_nowait(cv2_image)
        except:
            self.get_logger().warning('[Static Camera Papa Node]: La cola de imagenes esta llena, se descarta la imagen actual')
    def processing_loop(self):
        """
        Hilo que procesa las imagenes de la cola para detectar la bolsa de papa
        """
        self.get_logger().info('[Static Camera Papa Node]: Hilo de procesamiento en ejecucion')
        
        


    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('[Static Camera Papa Node]: Deteniendo hilos de procesamiento')
        self.stop_event.set()
        self.processing_thread.join()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        self.get_logger().info('[Static Camera Papa Node]: Hilos detenidos')

def main(args=None):
    rclpy.init(args=args)
    static_camera_papa_node = StaticCameraPapaNode()
    rclpy.spin(static_camera_papa_node)
    static_camera_papa_node.destroy_threads()
    static_camera_papa_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
