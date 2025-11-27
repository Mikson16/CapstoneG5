#!/usr/bin/env python3

"""
Nodo que recibe la imagen de la camara estatica y la procesa para identificacion de elemento extra dentro del rango de trabajo y mandar la señal de alerta para que sea recibida por el arduino

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
import cv2 as cv
from cv_bridge import CvBridge
from threading import Thread
from time import sleep
import numpy as np
from threading import Thread, Event
from queue import Queue, Empty

class StaticCameraEmergencyNode(Node):
    def __init__(self):
        super().__init__('static_camera_emergency_node')
        self.get_logger().info(f'[Static Camera Emergency Node]: ha sido iniciado')

        # Colas
        self.img_q = Queue(maxsize=5)
        self.stop_event = Event()

        #
        self.result_orange = None
        self.orange_mask = None

        # suscriptor
        self.subscription = self.create_subscription(Image, 'static_camera/image_raw', self.queue_callback, 10)

        self.publisher = self.create_publisher(Int16MultiArray, 'emergency/msg', 10)

        # Bridge
        self.bridge = CvBridge()

        # Hilo de procesamiento
        self.processing_thread = Thread(target=self.processing_loop)
        self.processing_thread.start()
    def queue_callback(self, msg):
        try:
            cv2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[Static Camera Papa Node]: Error al convertir la imagen: {e}')
            return
        
        # Ingresar a la cola
        try:
            self.img_q.put_nowait(cv2_image)
        except:
            self.get_logger().warning('La cola de imagenes esta llena, se descarta la imagen actual')
    def processing_loop(self):

        while not self.stop_event.is_set():

            try:
                frame = self.img_q.get(timeout=0.5)

                frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

                
                    # Naranjo
                # Se agrega el naranjo y se cambia por el azul ya que la manguera de la bomba es azul
                # DESCOMENTAR EN CASO QUE SE USE EL NARANJO
                lower_orange = np.array([12,  120,  120])
                upper_orange = np.array([20, 255, 255])

                orange_mask = cv.inRange(frame_hsv, lower_orange, upper_orange)

                kernel = cv.getStructuringElement(cv.MORPH_RECT, (35, 35))
                mask = cv.morphologyEx(orange_mask, cv.MORPH_CLOSE, kernel)

                contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

                if not contours:
                    continue
                cv.imshow('find orange', mask)
                cv.waitKey(1)
            except Empty:
                continue
            except Exception as e:
                self.get_logger().warning(f'Error {e}')
                self.get_logger().debug(traceback.format_exc())
                continue
    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('[Static Camera Robot Node]: Deteniendo hilos de procesamiento')
        self.stop_event.set()
        self.processing_thread.join()
        try:
            self.processing_thread.join(timeout=1.0)
            cv.destroyAllWindows()
        except:
            pass
        self.get_logger().info('Hilos detenidos')
def main(args=None):
    rclpy.init(args=args)
    static_camera_emergency = StaticCameraEmergencyNode()
    rclpy.spin(static_camera_emergency)
    static_camera_emergency.destroy_threads()
    static_camera_emergency.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
