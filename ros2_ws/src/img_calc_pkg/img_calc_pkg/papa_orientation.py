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
from math import atan2, degrees


class PapaOrientationNode(Node):
    def __init__(self):
        super().__init__('papa_orientation_node')

        self.get_logger().info('[Papa Orientation Node]: ha sido iniciado')

        # Armar colas
        self.papa_bbox_q = Queue(maxsize=5)
        self.color_bbox_q = Queue(maxsize=5)
        self.stop_event = Event()

        self.publisher = self.create_publisher(Int16MultiArray, 'orientation/papa_orientation', 10)

        # Crear suscriptor al mensaje de la bounding box de la bolsa de papa
        self.subscription = self.create_subscription(Int16MultiArray, 'static_camera/min_bbox', self.papa_bbox_q_callback, 10) 

        # Crear suscriptor que reciba las coordenadas del eslabon
        self.min_color_bbox = self.create_subscription(Int16MultiArray, 'static_camera_robot/min_bbox_coord', self.color_bbox_q_callback, 10)# esto no esta escuchando
        # Crear hilo de procesamiento
        self.processing_thread = Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

        # Color bbox
        # self.orange_bbox = None
        self.green_bbox = None
        self.red_bbox = None

        # bbox de la bolsa
        self.papa_bbox = None

        # Orientacion de la bolsa respecto al segundo eslabon
        self.ang_bolsa = None

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

                # Si se usa el naranjo descomentar esta parte:
                # self.orange_bbox = data_color[0:8]
                # self.green_bbox = data_color[8:16]
                # self.red_bbox = data_color[16: 24]
                self.green_bbox = data_color[0:8]
                self.red_bbox = data_color[8:16]
                self.papa_bbox = data_papa

                # self.get_logger().info(f' bbox {self.orange_bbox}, {self.green_bbox}, {self.red_bbox}')
                # # Obtener valores de la papa
                # self.get_logger().info(f'Info de la papa {data_papa}')

                ## Ahora el procesamiento, obtener el angulo de la papa respecto a los colores
                # Sacando la linea central vertical de la bolsa de papa
                pts = np.array([[int(data_papa[i]), int(data_papa[i+1])] for i in range(0, 8, 2)], dtype=int)
                green_pts = np.array([[int(self.green_bbox[i]), int(self.green_bbox[i+1])] for i in range(0, 8, 2)], dtype=int)
                red_pts = np.array([[int(self.red_bbox[i]), int(self.red_bbox[i+1])] for i in range(0, 8, 2)], dtype=int)

                center = pts.mean(axis = 0)
                cx = int(round(float(center[0])))
                cy = int(round(float(center[1])))

                green_center = green_pts.mean(axis=0)
                red_center = red_pts.mean(axis=0)
                # Centro verde
                gcx = int(round(green_center[0]))
                gcy = int(round(green_center[1]))
                # Centro rojo
                rcx = int(round(red_center[0]))
                rcy = int(round(red_center[1]))

                #Angulo de los colores
                # El orden de los colores es verde-rojo
                dx = rcx - gcx
                dy = rcy - gcy
                theta_robot = atan2(dy, dx)

                dist_1 = float(np.linalg.norm(pts[0] - pts[1]))
                dist_2 = float(np.linalg.norm(pts[1] - pts[2]))

                self.papa_bbox = pts
                self.get_logger().info(f'Centro promedio: {cx, cy}, distancias: {dist_1, dist_2}')

                if dist_1 > dist_2:
                    # significa que p1 con p2 y p3 con p4 son los lados mas grandes
                    # self.get_logger().info(f' el punto 1 y 2 con los 3 y 4 son los lados mas grandes {dist_1}')
                    dx_bolsa = pts[0][0] - pts[1][0]
                    dy_bolsa = pts[1][0] - pts[1][1]

                else:
                    # significa que p2 con p3 y p4 con p1 son los lados mas grandes 
                    # self.get_logger().info(f' El punto 2 y 3 con los 4 y 1 son los lados mas grandes {dist_2}')
                    dx_bolsa = pts[1][0] - pts[2][0]
                    dy_bolsa = pts[1][1] - pts[2][1]
                
                theta_bolsa = atan2(dy_bolsa, dx_bolsa)

                # Obtener diferencia
                delta = theta_bolsa - theta_robot

                # Convertir a grados
                grados = degrees(delta)
                # Normalizar entre -180 y 180
                grados = (grados + 180) % 360 - 180

                # Simetria
                if grados > 90:
                    grados -= 180
                elif grados < -90:
                    grados += 180
                
                self.get_logger().info(f"GRADOS: {grados}")
                self.ang_bolsa = int(round(grados))

                try:
                    msg = Int16MultiArray()
                    msg.data = [self.ang_bolsa]
                    self.get_logger().info(f'El mensaje que se esta enviando es {self.ang_bolsa}')
                    self.publisher.publish(msg)
                except Exception as e:
                    self.get_logger().warning(f"No se pudo publicar la orientacion la orientacion: {e}")

            except Full:
                self.get_logger().warning(f'Cola llena')
                self.ang_bolsa = None
            except Empty:
                # self.get_logger().info(f'Cola vacia')
                data_color = None
                self.ang_bolsa = None
            except Exception as e:
                self.get_logger().warning(f'[Papa Orientation Node] Problema al obtener bbox de los colores: {e}')
                self.get_logger().debug(traceback.format_exc())
                self.ang_bolsa = None
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


# https://stackoverflow.com/questions/35749246/python-atan-or-atan2-what-should-i-use
