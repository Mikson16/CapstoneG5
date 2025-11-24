#!/usr/bin/env python3

"""
Nodo que recibe las coordenadas 2D de la bolsa en el sistema del robot y calcula la cinematica inversa, posteriormente se envia al nodo de comunicacion, para ser enviada serialmente

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from threading import Thread
from time import sleep
import numpy as np
from threading import Thread, Event
from queue import Queue, Empty, Full
import traceback
from math import atan2, degrees, sqrt, pi, acos, cos, tan, sin, atan

class InvKinematicsNode(Node):
    def __init__(self):
        super().__init__('inv_kinematics_node')
        self.get_logger().info(f'[Inv Kinematics Node]: ha sido iniciado')

        # armar colas
        self.bag_coord_q = Queue(maxsize=1)
        self.stop_event = Event()

        # suscriptores y publicadores
        self.publisher = self.create_publisher(Int16MultiArray, 'inv_kinematics/angles', 10)

        self.subscription = self.create_subscription(Int16MultiArray, 'bag_coord_trans/new_coords', self.queue_coord_callback, 10)

        # Parametros
        self.coords = None # coordenadas x, y del centro de la bolsa
        self.largo_1 = 40.8 # cm  #  Largo del eslabon 1
        self.largo_2 = 21.0 #cm  # Largo del eslabon 2

        # Armar hilos
        self.processing_thread = Thread(target = self.processing_loop, daemon = False)
        self.processing_thread.start()



    def queue_coord_callback(self, msg):
        try:
            data = msg.data
            self.bag_coord_q.put_nowait(data)

        except Full:
            self.get_logger().info(f'La cola esta llena')
        except Exception as e:
            self.get_logger().warning(f'Cola de bag coord con problema: {e}')
            self.get_logger().debug(traceback.format_exc())      
    
    def processing_loop(self):
        while not self.stop_event.is_set():
            try:
                # obtener data de la cola
                self.coords = self.bag_coord_q.get_nowait()
                x = self.coords[0]
                y = self.coords[1]

                h = sqrt(x**2 + y**2)
                beta = acos((largo_1**2 + h**2 + largo_2 ** 2) / (2 * largo_1 * h))
                #beta = acos((largo_1**2 + h**2 + largo_2 ** 2) / (2 * largo_1 * h))
                gamma = atan(x / y)

                q1 = gamma - beta # Angulo de base / eslabon 1
                q2 = acos(( h**2 - largo_1**2 - largo_2**2) / 2*largo_1*largo_2) # angulo segundo eslabon

                self.get_logger().info(f'Angulos obtenidos {q1, q2}, para las coordenadas objetivo {x, y}')
            except Empty:
                pass # completar
            except Exception as e:
                self.get_logger().warning(f'Problemas en el loop de procesamiento: {e}')
                self.get_logger().debug(traceback.format_exc())
                continue
    
    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('Deteniendo hilos de procesamiento')
        self.stop_event.set()
        try:
            self.processing_thread.join(timeout=1.0)
        except:
            pass
        self.get_logger().info('Hilos detenidos')  

def main(args=None):
    rclpy.init(args=args)
    inv_kinematics_node = InvKinematicsNode()
    rclpy.spin(inv_kinematics_node)
    inv_kinematics_node.destroy_threads()
    inv_kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
