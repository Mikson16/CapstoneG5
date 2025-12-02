#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import cv2
import numpy as np
import traceback

class BagCoordTransNode(Node):
    def _init_(self):
        super()._init_('bag_coord_trans_node')
        self.get_logger().info('[Transform Node]: Iniciado con Corrección Radial.')

        # --- 1. CONFIGURACIÓN FÍSICA ---
        self.ALTURA_CAMARA_MM = 1140.0 # 114 cm
        self.ALTURA_PAPA_MM = 40.0     # Altura estimada de la bolsa/papa (ajustar si es muy gorda)
        
        # Factor base (en el centro de la imagen)
        self.MM_PER_PIXEL = 0.8797 

        self.IMG_CENTER_X = 320.0 
        self.IMG_CENTER_Y = 240.0 

        self.ROBOT_OFFSET_X = 302.0 
        self.ROBOT_OFFSET_Y = 286.0 

        # --- 2. FACTORES DE CORRECCIÓN (AQUI ESTA LA MAGIA) ---
        
        # K_DISTORTION: Coeficiente de corrección radial.
        # Rango típico: entre -1e-6 y 1e-6.
        # Si el robot se queda CORTO en los bordes -> Prueba valores POSITIVOS (ej: 2e-6)
        # Si el robot se pasa de LARGO en los bordes -> Prueba valores NEGATIVOS (ej: -2e-6)
        self.K_DISTORTION = 0.0 # <--- EMPIEZA EN 0.0 Y AJUSTA
        
        self.SWAP_XY = False      
        self.INVERT_X = True      
        self.INVERT_Y = False     

        self.publisher = self.create_publisher(Int16MultiArray, 'bag_coord_trans/new_coords', 10)
        self.subscriber = self.create_subscription(Int16MultiArray, 'static_camera/papa_coord', self.coord_callback, 10)

    def coord_callback(self, msg):
        try:
            raw_u = float(msg.data[0])
            raw_v = float(msg.data[1])

            if raw_u == 0 and raw_v == 0: return

            # PASO 1: Centrar coordenadas (Relativo al centro óptico)
            u_rel = (raw_u - self.IMG_CENTER_X)
            v_rel = (raw_v - self.IMG_CENTER_Y)

            # PASO 2: Corrección de Distorsión Radial (Lente)
            # Calculamos el radio al cuadrado (r^2) en pixeles
            r2 = u_rel*2 + v_rel*2
            
            # Factor de corrección polinómica: f = 1 + k*r^2
            radial_factor = 1.0 + (self.K_DISTORTION * r2)
            
            u_undistorted = u_rel * radial_factor
            v_undistorted = v_rel * radial_factor

            # PASO 3: Corrección de Paralaje (Altura del objeto)
            # Triangulación: x_real = x_img * (H_cam - h_obj) / H_cam
            # Esto corrige que el objeto parezca estar más lejos solo por ser alto
            height_factor = (self.ALTURA_CAMARA_MM - self.ALTURA_PAPA_MM) / self.ALTURA_CAMARA_MM
            
            u_corrected = u_undistorted * height_factor
            v_corrected = v_undistorted * height_factor

            # PASO 4: Convertir a MM (Escalado Lineal)
            val_x = u_corrected * self.MM_PER_PIXEL
            val_y = v_corrected * self.MM_PER_PIXEL

            # PASO 5: Transformación de Ejes (Robot)
            if self.SWAP_XY: val_x, val_y = val_y, val_x
            if self.INVERT_X: val_x = -val_x
            if self.INVERT_Y: val_y = -val_y

            robot_x = int(self.ROBOT_OFFSET_X + val_x)
            robot_y = int(self.ROBOT_OFFSET_Y + val_y)

            self.get_logger().info(f'Raw:({raw_u},{raw_v}) -> Corregido:({robot_x},{robot_y}) | K={self.K_DISTORTION}')
            
            out_msg = Int16MultiArray()
            out_msg.data = [robot_x, robot_y]
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.get_logger().debug(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = BagCoordTransNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
