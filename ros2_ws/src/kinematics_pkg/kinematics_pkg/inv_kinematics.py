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
import math
import cv2

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
        self.get_logger().info(f'Se crearon los publicadores y pulicadores')

        # Parametros
        self.coords = None # coordenadas x, y del centro de la bolsa
        self.largo_1 = 400.0 # mm  #  Largo del eslabon 1
        self.largo_2 = 210.0  #mm  # Largo del eslabon 2

        self.origen_base = 0.0 # None # en radianes
        self.origen_eslabon = 0.0 # None # en radianes
        # # Estos son 0 porque el origen del robot se considera el origen del sistema
        # self.OFFSET_CAMARA_X = 302.0 # mm
        # self.OFFSET_CAMARA_Y = 286.0 # mm

        # self.factor_resolucion = 0.8797 # Calcular
        # self.centro_camara_X = float(520/2)
        # self.centro_camara_Y = float(480/2)

        # self.ang_desp_max_eslabon = np.pi * 3/2
        # self.ang_desp_max_base = np.pi * 3/2
        # Alcance máximo para validación
        self.max_reach = self.largo_1 + self.largo_2

        # --- CONFIGURACIÓN DEL MAPA VISUAL ---
        self.map_w = 800
        self.map_h = 800
        self.scale = 0.8  # Zoom para que quepa en pantalla (1 px = 1/0.8 mm)
        
        # Origen visual (Donde dibujamos la base (0,0) en la ventana)
        # Lo ponemos abajo al centro para tener espacio arriba y a los lados
        self.draw_origin_x = 400
        self.draw_origin_y = 700

        # Armar hilos
        self.processing_thread = Thread(target = self.processing_loop, daemon = False)
        self.processing_thread.start()

    def queue_coord_callback(self, msg):
        try:
            data = list(msg.data)
            # self.get_logger().info(f'la data que esta llegando es: {data}')
            self.bag_coord_q.put_nowait(data)

        except Full:
            self.get_logger().info(f'La cola esta llena')
        except Exception as e:
            self.get_logger().warning(f'Cola de bag coord con problema: {e}')
            self.get_logger().debug(traceback.format_exc())     

    def draw_robot_arm(self, q1_rad, q2_rad, target_x, target_y):
        """
        Dibuja el esqueleto del robot basado en los ángulos calculados
        """
        # Crear lienzo negro
        canvas = np.zeros((self.map_h, self.map_w, 3), dtype=np.uint8)

        # --- 1. Calcular posición del CODO (Forward Kinematics parcial) ---
        # x_codo = L1 * cos(q1)
        # y_codo = L1 * sin(q1)
        elbow_x_mm = self.largo_1 * math.cos(q1_rad)
        elbow_y_mm = self.largo_1 * math.sin(q1_rad)

        # --- 2. Calcular posición de la MUÑECA (FK total) ---
        # x_muneca = x_codo + L2 * cos(q1 + q2)
        # y_muneca = y_codo + L2 * sin(q1 + q2)
        wrist_x_mm = elbow_x_mm + self.largo_2 * math.cos(q1_rad + q2_rad)
        wrist_y_mm = elbow_y_mm + self.largo_2 * math.sin(q1_rad + q2_rad)

        # --- 3. Convertir a Pixeles de Dibujo ---
        # Eje Y del robot crece hacia ARRIBA/ABAJO segun tu definicion, 
        # pero en imagen Y crece ABAJO. Invertimos Y para dibujar intuitivamente.
        
        # BASE
        bx = self.draw_origin_x
        by = self.draw_origin_y

        # CODO
        ex = int(self.draw_origin_x + (elbow_x_mm * self.scale))
        ey = int(self.draw_origin_y - (elbow_y_mm * self.scale)) # Restamos Y para ir "arriba" en la pantalla

        # MUÑECA (Calculada con angulos)
        wx = int(self.draw_origin_x + (wrist_x_mm * self.scale))
        wy = int(self.draw_origin_y - (wrist_y_mm * self.scale))

        # TARGET REAL (El punto que recibimos)
        tx = int(self.draw_origin_x + (target_x * self.scale))
        ty = int(self.draw_origin_y - (target_y * self.scale))

        # --- 4. DIBUJAR ---
        
        # Ejes
        cv2.line(canvas, (bx, by), (bx+50, by), (100,100,100), 1) # X axis
        cv2.line(canvas, (bx, by), (bx, by-50), (100,100,100), 1) # Y axis

        # Eslabón 1 (Base -> Codo) - AZUL
        cv2.line(canvas, (bx, by), (ex, ey), (255, 100, 0), 4)
        
        # Eslabón 2 (Codo -> Muñeca) - ROJO
        cv2.line(canvas, (ex, ey), (wx, wy), (0, 0, 255), 4)

        # Articulaciones
        cv2.rectangle(canvas, (bx-10, by-10), (bx+10, by+10), (0, 255, 0), -1) # Base Verde
        cv2.circle(canvas, (ex, ey), 8, (0, 255, 255), -1) # Codo Amarillo
        cv2.circle(canvas, (wx, wy), 5, (255, 255, 255), -1) # Muñeca Blanca

        # Objetivo Deseado (Target) - Cruz Roja pequeña
        cv2.drawMarker(canvas, (tx, ty), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)

        # Texto Informativo
        info_ang = f"Q1: {math.degrees(q1_rad):.1f} deg | Q2: {math.degrees(q2_rad):.1f} deg"
        info_pos = f"Target: ({target_x:.0f}, {target_y:.0f})"
        
        cv2.putText(canvas, "VISUALIZADOR CINEMATICA", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(canvas, info_ang, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 1)
        cv2.putText(canvas, info_pos, (20, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)

        cv2.imshow("Debug IK Robot", canvas)
        cv2.waitKey(1)

    def processing_loop(self):
        """
        Procesamiento de cinemática inversa con Transformación de Coordenadas
        """
        while not self.stop_event.is_set():
            try:
                coords_camara = self.bag_coord_q.get_nowait()
                x_cam_trans = coords_camara[0]
                y_cam_trans = coords_camara[1]
                
                # self.get_logger().info(f"Cam: ({x_cam_trans}, {y_cam_trans})")

                h = np.sqrt(x_cam_trans**2 + y_cam_trans**2)

                # Verificacion de alcance
                if h > self.max_reach or h < abs(self.largo_1 - self.largo_2):
                    self.get_logger().warning(f'Fuera de rango. Dist: {h:.1f}')
                    # Igual dibujamos donde esta el objetivo aunque no lleguemos
                    try:
                        self.draw_robot_arm(0, 0, x_cam_trans, y_cam_trans)
                    except: pass
                    continue

                # q2
                numerador = self.largo_1**2 + self.largo_2**2 - h**2
                denominador = 2 * self.largo_1 * self.largo_2
                
                cos_q2_int = numerador / denominador
                cos_q2_int = np.clip(cos_q2_int, -1.0, 1.0) # Evitar errores numericos
                
                q2_interno = np.arccos(cos_q2_int)
                q2 = np.pi - q2_interno 

                # q1
                gamma = np.arctan2(y_cam_trans, x_cam_trans) 

                # Ley de cosenos para beta
                if h == 0:
                    continue

                num_beta = self.largo_1**2 + h**2 - self.largo_2**2
                den_beta = 2 * self.largo_1 * h
                
                cos_beta = num_beta / den_beta
                cos_beta = np.clip(cos_beta, -1.0, 1.0)
                beta = np.arccos(cos_beta)

                q1 = gamma - beta

                q1_final = q1 + self.origen_base
                q2_final = q2 + self.origen_eslabon
                
                # Enviar
                msg = Int16MultiArray()

                deg_q1 = np.degrees(q1_final)
                deg_q2 = np.degrees(q2_final)
                
                data = [int(round(deg_q1)), int(round(deg_q2))]
                msg.data = data
                self.publisher.publish(msg)
                # try:
                #     self.draw_robot_arm(q1_final, q2_final, x_cam_trans, y_cam_trans)
                # except Exception as e:
                #     self.get_logger().warning(f"Error dibujo: {e}")

                # self.get_logger().info(f'Enviando Grados desde la cinematica inversa: {data}')

            except Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error IK: {e}')
                self.get_logger().error(traceback.format_exc())
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
