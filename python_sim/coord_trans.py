#!/usr/bin/env python3

"""
Nodo ROBUSTO de Transformación de Coordenadas (Imagen -> Robot SCARA)
Permite inversión de ejes, rotación y escalado preciso.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import cv2
import numpy as np
import traceback

class BagCoordTransNode(Node):
    def _init_(self):
        super()._init_('bag_coord_trans_node')
        self.get_logger().info('[Transform Node]: Iniciado. Esperando coordenadas...')

        # --- 1. CALIBRACIÓN FÍSICA (AJUSTAR AQUÍ) ---
        
        # Factor de escala (mm por pixel)
        # Calculado con altura 115cm. Si el ancho real visto por la camara es ~56cm en 640px -> 0.875
        self.MM_PER_PIXEL = 0.8797 

        # Centro de la imagen (Pixel)
        self.IMG_CENTER_X = 320.0 # Asumiendo resolución 640x480
        self.IMG_CENTER_Y = 240.0 

        # POSICIÓN DE LA CÁMARA RESPECTO AL ROBOT
        # Mide con una regla desde la BASE DEL ROBOT hasta el punto en la mesa
        # que cae justo en el centro de la imagen.
        self.ROBOT_OFFSET_X = 302.0 # mm (Distancia X desde base a centro imagen)
        self.ROBOT_OFFSET_Y = 286.0 # mm (Distancia Y desde base a centro imagen)

        # --- 2. CONFIGURACIÓN DE EJES (LA CLAVE DEL PROBLEMA) ---
        # Cambia estos True/False observando los resultados
        
        self.SWAP_XY = False      # Pon True si al moverte en X imagen, cambia el Y robot
        self.INVERT_X = True      # Pon True si al ir a la derecha en imagen, el X robot baja
        self.INVERT_Y = False     # Pon True si al ir abajo en imagen, el Y robot baja

        # -----------------------------------------------------------

        # Publicadores y suscriptores
        self.publisher = self.create_publisher(Int16MultiArray, 'bag_coord_trans/new_coords', 10)
        self.subscriber = self.create_subscription(Int16MultiArray, 'static_camera/papa_coord', self.coord_callback, 10)

        # Para debug visual
        self.debug_map_pub = None # Opcional: Podrías publicar la imagen de debug
    
    def coord_callback(self, msg):
        """
        Se ejecuta cada vez que la cámara detecta algo.
        """
        try:
            # 1. Obtener coordenadas crudas (Pixeles)
            raw_u = float(msg.data[0]) # Pixel X
            raw_v = float(msg.data[1]) # Pixel Y

            if raw_u == 0 and raw_v == 0:
                return # Filtrar ceros si es necesario

            # 2. Centrar coordenadas (Origen en el centro de la imagen)
            # u_centrado positivo = derecha del centro
            # v_centrado positivo = abajo del centro
            u_centered = (raw_u - self.IMG_CENTER_X) * self.MM_PER_PIXEL
            v_centered = (raw_v - self.IMG_CENTER_Y) * self.MM_PER_PIXEL

            # 3. Aplicar Transformación de Ejes (Matriz de Rotación simplificada)
            # Esto alinea el sistema de la cámara con el del robot
            
            val_x = u_centered
            val_y = v_centered

            # Intercambio (si la cámara está rotada 90 grados)
            if self.SWAP_XY:
                val_x, val_y = val_y, val_x
            
            # Inversión de dirección (según hacia donde apunte el eje del robot)
            if self.INVERT_X:
                val_x = -val_x
            
            if self.INVERT_Y:
                val_y = -val_y

            # 4. Traslación al Origen del Robot
            # Sumamos el offset para llevar el (0,0) de la imagen al (302, 286) del robot
            robot_x = int(self.ROBOT_OFFSET_X + val_x)
            robot_y = int(self.ROBOT_OFFSET_Y + val_y)

            # 5. Publicar y Debug
            self.get_logger().info(f'In(Px):{raw_u:.0f},{raw_v:.0f} -> Out(mm): X={robot_x}, Y={robot_y}')
            
            out_msg = Int16MultiArray()
            out_msg.data = [robot_x, robot_y]
            self.publisher.publish(out_msg)

            # Visualización (Opcional, abre ventana en el PC del robot)
            self.show_debug_window(robot_x, robot_y, raw_u, raw_v)

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

if _name_ == '_main_':
    main()
