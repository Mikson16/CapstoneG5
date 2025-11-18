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
from queue import Queue, Empty, Full
import traceback


class PapaOrientationNode(Node):
    def __init__(self):
        super().__init__('papa_orientation_node')

        self.get_logger().info('[Papa Orientation Node]: ha sido iniciado')

        # Armar colas
        self.papa_bbox_q = Queue(maxsize=5)
        self.color_bbox_q = Queue(maxsize=5)
        self.stop_event = Event()

        # Crear suscriptor al mensaje de la bounding box de la bolsa de papa
        self.subscription = self.create_subscription(Int16MultiArray, 'static_camera/min_bbox', self.papa_bbox_q_callback, 10) 

        # Crear suscriptor que reciba las coordenadas del eslabon
        self.min_color_bbox = self.create_subscription(Int16MultiArray, 'static_camera_robot/min_bbox_coord', self.color_bbox_q_callback, 10)# esto no esta escuchando
        # Crear hilo de procesamiento
        self.processing_thread = Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

        # Color bbox
        self.orange_bbox = None
        self.green_bbox = None
        self.red_bbox = None

        # bbox de la bolsa
        self.papa_bbox = None

    def papa_bbox_q_callback(self, msg):
        try:
            data = list(msg.data)
            self.papa_bbox_q.put_nowait(data)
        except Full:
            self.get_logger().warning('papa_bbox_q llena: descartando mensaje')
        except Exception as e:
            self.get_logger().warning(f'Cola de bbox papa con problema: {e}')
            self.get_logger().debug(traceback.format_exc())

    def color_bbox_q_callback(self, msg):
        try:
            data = list(msg.data)
            self.color_bbox_q.put_nowait(data)
        except Full:
            self.get_logger().warning('color_bbox_q llena: descartando mensaje')
        except Exception as e:
            self.get_logger().warning(f'Cola de bbox color con problema: {e}')
            self.get_logger().debug(traceback.format_exc())


    def processing_loop(self):
        # self.get_logger().info(f'mensaje {msg.data}')
        while not self.stop_event.is_set():

            try:
                #Obtener valores de los colores
                data_color = self.color_bbox_q.get_nowait()
                data_papa = self.papa_bbox_q.get_nowait() 

                self.orange_bbox = data_color[0:8]
                self.green_bbox = data_color[8:16]
                self.red_bbox = data_color[16: 24]
                self.papa_bbox = data_papa

                self.get_logger().info(f' bbox {self.orange_bbox}, {self.green_bbox}, {self.red_bbox}')
                # Obtener valores de la papa
                self.get_logger().info(f'Info de la papa {data_papa}')
            except Full:
                self.get_logger().warning(f'Cola llena')
            except Empty:
                # self.get_logger().info(f'Cola vacia')
                data_color = None
            except Exception as e:
                self.get_logger().warning(f'[Papa Orientation Node] Problema al obtener bbox de los colores: {e}')
                self.get_logger().debug(traceback.format_exc())
                continue

    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('Deteniendo hilos de procesamiento')
        self.stop_event.set()
        self.processing_thread.join(timeout=1.0)
        try:
            cv2.destroyAllWindows()
        except:
            pass
        self.get_logger().info('Hilos detenidos')          

def main(args=None):
    rclpy.init(args=args)
    papa_orientation_node = PapaOrientationNode()
    rclpy.spin(papa_orientation_node)
    papa_orientation_node.destroy_threads()
    papa_orientation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
