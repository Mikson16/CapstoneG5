#!/usr/bin/env python3

"""
Nodo que recibe las coordenadas del min bounding box de la papa, los eslabones 
y obtiene la orientacion de la bolsa respecto a los eslabones

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


class PapaOrientationNode(Node):
    def __init__(self):
        super().__init__('papa_orientation_node')

        self.get_logger().info('[Papa Orientation Node]: ha sido iniciado')

        # Crear suscriptor al mensaje de la bounding box
        self.subscription = self.create_subscription(Int16MultiArray, 'static_camera/min_bbox', self.min_bbox_callback, 10)

        # Crear suscriptor que reciba las coordenadas del eslabon
        self.min_color_bbox = self.create_subscription(Int16MultiArray, 'static_camera_robot/min_bbox_coord', min_color_bbox_callback, 10)

        # Color bbox
        self.orange_bbox = None
        self.green_bbox = None
        self.red_bbox = None


    def min_bbox_callback(self, msg):
        pass

    def min_color_bbox_callback(self, msg):
        try:
            self.orange_bbox = msg[0]
            self.green_bbox = msg[1]
            self.red_bbox = msg[2]

        except Exception as e:
            self.get_logger().warning(f'[Papa Orientation Node] Problema al obtener bbox de los colores: {e}')
            