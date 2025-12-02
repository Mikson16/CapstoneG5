#!/usr/bin/env python3

"""
Nodo para iniciar la camara estatica y publicar las imagenes que captura.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from threading import Thread
from time import sleep


class StaticCameraNode(Node):

    def __init__(self):
        super().__init__('static_camera_node')
        self.publisher = self.create_publisher(Image, 'static_camera/image_raw', 10)
        
        # Parametros
        self.crop_der = 70
        self.crop_izq = 50
        try:
            self.capture = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Ajustar indice segun la camara estatica, si esta en el USB 3 los indices son 2 y 3
        except Exception as e:
            self.get_logger().error(f'Error al abrir la camara: {e}')
            # self.capture = cv2.VideoCapture(0)  
            
        self.bridge = CvBridge()

        sleep(2)  # Esperar a que la camara se inicialice
        self.get_logger().info('[Static Camera Node]: ha sido iniciado')
        self.timer = self.create_timer(0.2, self.show_capture_callback)  # Publicar cada 0.2 segundos, 5Hz

    def show_capture_callback(self):
        ret, frame = self.capture.read()
        if ret:
            # Acortar la imagen para obtener solo el area de trabajo
            
            frame = frame[:, self.crop_der : -self.crop_izq, :]
            self.get_logger().info(f'El tamaño de la imagen es {frame.shape}') # es de 480, 640, 3
            center = (int(frame.shape[1] / 2), int(frame.shape[0] / 2))
            cv2.circle(frame, center, 1, (0, 255, 0), -1)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            cv2.imshow('Static Camera Node', frame)
            cv2.waitKey(1)
            self.publisher.publish(msg)
        else:
            self.get_logger().error('No se pudo capturar la imagen de la camara')


def main(args=None):
    rclpy.init(args=args)
    static_camera_node = StaticCameraNode()
    rclpy.spin(static_camera_node)
    static_camera_node.capture.release()
    cv2.destroyAllWindows()
    static_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
