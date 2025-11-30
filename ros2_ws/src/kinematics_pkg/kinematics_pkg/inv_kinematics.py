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

        self.origen_base = 0.0 # None # en radianes
        self.origen_eslabon = 0.0 # None # en radianes
        # Estos son 0 porque el origen del robot se considera el origen del sistema
        self.OFFSET_CAMARA_X = 302.0 # mm
        self.OFFSET_CAMARA_Y = 286.0 # mm

        self.factor_resolucion = 0.8797 # Calcular
        self.centro_camara_X = float(520/2)
        self.centro_camara_Y = float(480/2)

        self.ang_desp_max_eslabon = np.pi * 3/2
        self.ang_desp_max_base = np.pi * 3/2

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
    """
    Procesamiento de cinemática inversa con Transformación de Coordenadas
    """
    while not self.stop_event.is_set():
        try:
            # 1. Obtener coordenadas relativas a la CAMARA
            # (Asumo que 'coords' viene en milímetros desde tu nodo de visión.
            #  Si viene en pixeles, debes multiplicar por tu factor de escala antes)
            coords_camara = self.bag_coord_q.get_nowait()
            x_cam = coords_camara[0]
            y_cam = coords_camara[1]


            #
            x_rel_mm = (x_cam - self.centro_camara_X) * self.factor_resolucion
            y_rel_mm = (y_cam - self.centro_camara_Y) * self.factor_resolucion
            #

            # ---------------------------------------------------------
            # 2. TRANSFORMACIÓN DE COORDENADAS (Cámara -> Robot)
            # ---------------------------------------------------------
            
            x = x_cam + self.OFFSET_CAMARA_X
            y = y_cam + self.OFFSET_CAMARA_Y
            
            # Debug: Ver si la coordenada tiene sentido
            self.get_logger().info(f"Cam: ({x_cam}, {y_cam}) -> Robot: ({x}, {y})")

            # ---------------------------------------------------------
            # 3. CÁLCULO DE CINEMÁTICA INVERSA (Corregido)
            # ---------------------------------------------------------
            
            # CORRECCIÓN 1: Usar **2 para elevar al cuadrado
            h = np.sqrt(x**2 + y**2)

            # Verificación de alcance
            if h > (self.largo_1 + self.largo_2) or h < np.abs(self.largo_1 - self.largo_2):
                self.get_logger().warning(f'Fuera de rango. Dist: {h:.2f} | Max: {self.largo_1 + self.largo_2}')
                continue 

            # --- Q2 (Codo) ---
            # CORRECCIÓN: **2 en lugar de *2
            numerador = self.largo_1**2 + self.largo_2**2 - h**2
            denominador = 2 * self.largo_1 * self.largo_2
            
            cos_q2_int = numerador / denominador
            cos_q2_int = np.clip(cos_q2_int, -1.0, 1.0) # Evitar errores numéricos
            
            q2_interno = np.arccos(cos_q2_int)
            q2 = np.pi - q2_interno 
            # NOTA: Si el codo dobla al revés, prueba: q2 = -(np.pi - q2_interno)

            # --- Q1 (Hombro) ---
            # CORRECCIÓN 2: Usar arctan2 para respetar cuadrantes
            gamma = np.arctan2(y, x) 

            # Ley de cosenos para beta (ángulo interno del triángulo base-hombro-muñeca)
            if h == 0:
                continue

            # CORRECCIÓN: **2 otra vez
            num_beta = self.largo_1**2 + h**2 - self.largo_2**2
            den_beta = 2 * self.largo_1 * h
            
            cos_beta = num_beta / den_beta
            cos_beta = np.clip(cos_beta, -1.0, 1.0)
            beta = np.arccos(cos_beta)

            q1 = gamma - beta

            # Ajuste de origen de los motores (Calibration offset)
            q1_final = q1 + self.origen_base
            q2_final = q2 + self.origen_eslabon
            
            # Convertir a Grados para verificar limites (o si tus limites son radianes, dejalo asi)
            # Asumo que tus limites self.ang_desp_max_base están en la misma unidad que q1/q2

            # ... (Tus verificaciones de limites aqui) ...

            # Enviar
            msg = Int16MultiArray()
            # Convertir radianes a grados para el Arduino? 
            # Ojo: np.arccos devuelve radianes. Si tu arduino espera grados:
            deg_q1 = np.degrees(q1_final)
            deg_q2 = np.degrees(q2_final)
            
            data = [int(round(deg_q1)), int(round(deg_q2))]
            msg.data = data
            self.publisher.publish(msg)

            self.get_logger().info(f'Enviando Grados: {data}')

        except Empty:
            pass
        except Exception as e:
            self.get_logger().error(f'Error IK: {e}')
            # traceback es util, importalo si no lo tienes
            # import traceback
            # self.get_logger().error(traceback.format_exc())


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
