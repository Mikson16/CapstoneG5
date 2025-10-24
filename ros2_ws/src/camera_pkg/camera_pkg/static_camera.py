#!/usr/bin/env python3

"""
Nodo para la camara estatica
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
        
        try:
            self.capture = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Si se esta en el computador, esto es 0 para la camara del computador, 1 para la camara externa, si se esta en la raspi 0 es la camara correcta
        except Exception as e:
            self.get_logger().error(f'Error al abrir la camara: {e}')
            self.capture = cv2.VideoCapture(0)  
            
        self.bridge = CvBridge()

        sleep(2)  # Esperar a que la camara se inicialice
        self.get_logger().info('Static Camera Node ha sido iniciado')
        self.timer = self.create_timer(0.1, self.show_capture_callback)  # Publicar cada 0.1 segundos

    def show_capture_callback(self):
        ret, frame = self.capture.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            cv2.imshow('Static Camera', frame)
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
