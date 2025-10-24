#!/usr/bin/env python3

"""
Nodo de levantamiento de la camara movil frontal y publicacion de las imagenes capturadas.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from threading import Thread
from time import sleep

class FrontCameraNode(Node):

    def __init__(self):
        super().__init__('front_camera_node')

        # Crear publicador para camara frontal
        self.publisher = self.create_publisher(Image, 'front_camera/image_raw', 10)

        # Inicializar captura de video
        try:
            self.capture = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Ajustar el indice segun la camara frontal si esta en  USB 4 los indices son 0 y 1
        except Exception as e:
            self.get_logger().error(f'Error al abrir la camara frontal: {e}')
            self.capture = cv2.VideoCapture(0)

        # Inicializar CvBridge
        self.bridge = CvBridge()

        #Delay para esperar a que la camara se inicialice
        sleep(2)
        self.get_logger().info('[Front Camera Node]: ha sido iniciado')
        self.timer = self.create_timer(0.1, self.show_capture_callback)  # Publicar cada 0.1 segundos

    def show_capture_callback(self):
        ret, frame = self.capture.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            cv2.imshow('Front Camera', frame)
            cv2.waitKey(1)
            self.publisher.publish(msg)
        else:
            self.get_logger().error('No se pudo capturar la imagen de la camara frontal')

def main(args=None):
    rclpy.init(args=args)
    front_camera_node = FrontCameraNode()
    rclpy.spin(front_camera_node)
    front_camera_node.capture.release()
    cv2.destroyAllWindows()
    front_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
