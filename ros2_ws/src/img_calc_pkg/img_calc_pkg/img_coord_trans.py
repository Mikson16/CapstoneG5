#!/usr/bin/env python3

"""
Nodo de Transformación de Coordenadas (Imagen -> Robot SCARA)
Permite inversion de ejes, rotacion y escalado preciso.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import cv2
import numpy as np
import traceback
from threading import Thread, Event
from queue import Queue, Empty
import math

class CorrectorCorr:
    def __init__(self):
        self.puntos_calibracion=[
            [12, 422, 20, 10], [39, 406, 0, 10], [397, 306, 60, 0], [428, 241, 50, 20], [221, 418, -60, 65], [61, 354, -50, 5], [407, 220, 40, -30], [345, 355, 0, 90], [95, 273, -60, 20], [280, 140, 0, -90], [-150, 266, -20, 0], [22, 488, 20, 10], [480, 158, 50, -100], [345, 329, 20, 60], [194, 432, 0, 40], [80, 449, -34, 10], 
            
            [-106, 492, 40, -5], [190, 382, 0, 50], [245, 381, 20, 75], [376, 250, 80, -45],
            [-230, 450, 118, -45], [179, 345, -40, 40], [300, 375, 28, 68], [105, 472, -25, 43], [25, 401, -42, 15], [-90, 358, 15, -15]
        ]
        self.radio_influencia = 300.0 #mm
        self.escalador = 1.0

    def corregir_corrd(self, x_in, y_in):
        peso_total = 0.0
        peso_x = 0.0
        peso_y = 0.0

        for point in self.puntos_calibracion:
            ancho_x, ancho_y, err_x, err_y = point

            dist = np.sqrt((x_in - ancho_x)**2 + (y_in - ancho_y)**2)

            # evitar el 0
            if dist < 0.1:
                dist = 0.1

            peso_gauss = np.exp(-(dist**2) / (2 * self.radio_influencia**2))

            peso_x += err_x * peso_gauss
            peso_y += err_y * peso_gauss
            peso_total += peso_gauss

            if peso_total == 0 :
                return x_in, y_in

            # aplicar correccion
            x = x_in + (peso_x / peso_total) * self.escalador
            y = y_in + (peso_y / peso_total) * self.escalador

            return x, y
class BagCoordTransNode(Node):
    def __init__(self):
        super().__init__('bag_coord_trans_node')
        self.get_logger().info('[Transform Node]: Iniciado. Esperando coordenadas...')
        
        self.corrector = CorrectorCorr()

        self.ALTURA_CAMARA_MM = 1140.0
        self.ALTURA_PAPA_MM = -100.0
        
        # Factor de escala (mm por pixel)
        # Calculado con altura 114cm
        self.scale = 0.877192 

        # Centro de la imagen
        self.IMG_CENTER_X = float(520/2) 
        self.IMG_CENTER_Y = float(480/2) 


        self.ROBOT_OFFSET_X = 310.0 
        self.ROBOT_OFFSET_Y = 286.0 

        # Desplazamiento base
        self.desp_x = 0#- 30.0 # 3 cm
        self.desp_y = 0#- 100.0 # 10 cm

        # FACTORES DE CORRECCION
        self.K_DISTORTION = 2.155e-5

        # Cambiar estos True/False observando los resultados
        
        self.SWAP_XY = False      # True si al mover en Xen la imagen, cambia el Y robot
        self.INVERT_X = True      # True si al ir a la derecha en la imagen, el X robot baja
        self.INVERT_Y = False     # True si al ir abajo en la imagen, el Y robot baja

        # Publicadores y suscriptores
        self.publisher = self.create_publisher(Int16MultiArray, 'bag_coord_trans/new_coords', 10)
        self.subscriber = self.create_subscription(Int16MultiArray, 'static_camera/papa_coord', self.coord_callback, 10)

        # Para debug visual
        self.debug_map_pub = None
    
    def coord_callback(self, msg):
        """
        Se ejecuta cada vez que la camara detecta algo.
        """
        try:
            # 1. Obtener coordenadas crudas (Pixeles)
            raw_u = float(msg.data[0]) # Pixel X
            raw_v = float(msg.data[1]) # Pixel Y

            if raw_u == 0 and raw_v == 0:
                return 

            # Comentar esta linea para no usar el corrector gaussiano
            u_corr, y_corr = self.corrector.corregir_corrd(raw_u, raw_v) 

            # 2. Centrar coordenadas
            # u_centrado positivo = derecha del centro
            # v_centrado positivo = abajo del centro
            u_centered = (u_corr - self.IMG_CENTER_X) 
            v_centered = (y_corr - self.IMG_CENTER_Y)

            # Correccion de distorcion por radio
            r2 = u_centered **2 + v_centered **2
            # Factor de correccion polinomica
            radial_factor = 1.0 + (self.K_DISTORTION * r2)

            u_sin_dist = u_centered * radial_factor
            v_sin_dist = v_centered * radial_factor

            # Correccion de paralaje
            factor_h = 1.1 #(self.ALTURA_CAMARA_MM - self.ALTURA_PAPA_MM) / self.ALTURA_CAMARA_MM

            u_correct = u_sin_dist * factor_h
            v_correct = v_sin_dist * factor_h

            # Escalado lineal
            x = u_correct * self.scale
            y = v_correct * self.scale


            # Intercambio (si la cámara está rotada 90 grados)
            if self.SWAP_XY:
                x, y = y, x
            
            # Inversión de dirección (según hacia donde apunte el eje del robot)
            if self.INVERT_X:
                x = -x
            
            if self.INVERT_Y:
                y = -y



            # 4. Traslacion al Origen del Robot
            robot_x = int(self.ROBOT_OFFSET_X + x - self.desp_x)
            robot_y = int(self.ROBOT_OFFSET_Y + y - self.desp_y)

            # 5. Publicar y Debug
            # self.get_logger().info(f'In(Px):{raw_u:.0f},{raw_v:.0f} -> Out(mm): X={robot_x}, Y={robot_y}')
            
            out_msg = Int16MultiArray()
            out_msg.data = [robot_x, robot_y]
            self.get_logger().info(f'\n Las coordenadas cartesianas a enviar son x {robot_x}, y {robot_y}')
            self.publisher.publish(out_msg)

            # Descomentar para visualizar
            # self.show_debug_window(robot_x, robot_y, raw_u, raw_v)

        except Exception as e:
            self.get_logger().error(f'Error procesando coordenadas: {e}')
            self.get_logger().debug(traceback.format_exc())

    def show_debug_window(self, rx, ry, px, py):
        """Dibuja un mapa visual para entender dónde cree el robot que está el objeto"""
        W, H = 600, 600
        canvas = np.zeros((H, W, 3), dtype=np.uint8)
        
        # Factor de escala visual (para que quepa en pantalla)
        scale = 0.5 
        origin_x, origin_y = 50, 550 # Dibujamos el origen robot abajo a la izquierda
        
        # Dibujar Ejes Robot
        cv2.arrowedLine(canvas, (origin_x, origin_y), (origin_x + 100, origin_y), (0,0,255), 2) # X Rojo
        cv2.putText(canvas, "X Robot", (origin_x + 80, origin_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        
        cv2.arrowedLine(canvas, (origin_x, origin_y), (origin_x, origin_y - 100), (0,255,0), 2) # Y Verde
        cv2.putText(canvas, "Y Robot", (origin_x - 20, origin_y - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # Dibujar Punto Objetivo
        # Convertir mm robot a pixeles de pantalla
        draw_x = int(origin_x + (rx * scale))
        draw_y = int(origin_y - (ry * scale)) # Y pantalla crece hacia abajo, Y robot hacia arriba (usualmente)

        cv2.circle(canvas, (draw_x, draw_y), 8, (0, 255, 255), -1)
        cv2.putText(canvas, f"Papa: {rx},{ry}", (draw_x+10, draw_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        cv2.imshow("Debug Coordenadas", canvas)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BagCoordTransNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
