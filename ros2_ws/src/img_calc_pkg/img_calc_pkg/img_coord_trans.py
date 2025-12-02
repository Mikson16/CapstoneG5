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
    def __init__(self):
        super().__init__('bag_coord_trans_node')
        self.get_logger().info('[Transform Node]: Iniciado. Esperando coordenadas...')

        # --- 1. CALIBRACIÓN FÍSICA (AJUSTAR AQUÍ) ---
        
        # Factor de escala (mm por pixel)
        # Calculado con altura 115cm. Si el ancho real visto por la camara es ~56cm en 640px -> 0.875
        self.MM_PER_PIXEL = 0.8797 

        # Centro de la imagen (Pixel)
        self.IMG_CENTER_X = float(520/2) # Asumiendo resolución 640x480
        self.IMG_CENTER_Y = float(480/2) 

        # POSICIÓN DE LA CÁMARA RESPECTO AL ROBOT
        # Mide con una regla desde la BASE DEL ROBOT hasta el punto en la mesa
        # que cae justo en el centro de la imagen.
        self.ROBOT_OFFSET_X = 302.0 # mm (Distancia X desde base a centro imagen)
        self.ROBOT_OFFSET_Y = 286.0 # mm (Distancia Y desde base a centro imagen)

        # Desplazamiento base
        self.desp_x = - 30.0 # 3 cm
        self.desp_y = - 100.0 # 10 cm

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
            robot_x = int(self.ROBOT_OFFSET_X + val_x - self.desp_x)
            robot_y = int(self.ROBOT_OFFSET_Y + val_y - self.desp_y)

            # 5. Publicar y Debug
            # self.get_logger().info(f'In(Px):{raw_u:.0f},{raw_v:.0f} -> Out(mm): X={robot_x}, Y={robot_y}')
            
            out_msg = Int16MultiArray()
            out_msg.data = [robot_x, robot_y]
            self.publisher.publish(out_msg)

            # Visualización (Opcional, abre ventana en el PC del robot)
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


# """
# Nodo que recibe las coordenadas de la imagen y transforma los pixeles a las coordenadas del robot, luego envia la informacion apra realizar la cinematica inversa
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int16MultiArray
# from threading import Thread
# from time import sleep
# import cv2
# import numpy as np
# from threading import Thread, Event
# from queue import Queue, Empty, Full
# import traceback
# from math import atan2, degrees


# class BagCoordTransNode(Node):
#     def __init__(self):
#         super().__init__('bag_coord_trans_node')
#         self.get_logger().info(f'[Bag Coord Transformation Node]: ha sido iniciado')

#         # Armar colas
#         self.bag_coord_q = Queue(maxsize=1)
#         self.stop_event = Event()

#         # Publicadores y suscriptores
#         self.publisher = self.create_publisher(Int16MultiArray, 'bag_coord_trans/new_coords', 10)

#         self.subscriber = self.create_subscription(Int16MultiArray, 'static_camera/papa_coord', self.queue_coord_callback, 10)

#         # Parametros
#         self.origen_base = 0.0 # None # en radianes
#         self.origen_eslabon = 0.0 # None # en radianes
#         # Estos son 0 porque el origen del robot se considera el origen del sistema
#         self.OFFSET_CAMARA_X = 302.0 # mm
#         self.OFFSET_CAMARA_Y = 286.0 # mm

#         self.factor_resolucion = 0.8797 # Calcular
#         self.centro_camara_X = float(520/2)
#         self.centro_camara_Y = float(480/2)

#         # Para el grafico de debug
#         self.map_width = 800
#         self.map_height = 800
#         self.scale_draw = 1.0 # 1 pixel = 1 mm para dibujar facil

#         self.origin_draw_x = 750
#         self.origin_draw_y = 50

#         # Armar hilos
#         self.processing_thread = Thread(target = self.processing_loop)
#         self.processing_thread.start()


#     def queue_coord_callback(self, msg):
#         try:
#             self.get_logger().info(f'Llego un mensaje, guardando en cola')
#             data = msg.data
#             self.bag_coord_q.put_nowait(data)

#         except Full:
#             self.get_logger().info(f'La cola esta llena')
#         except Exception as e:
#             self.get_logger().warning(f'Cola de bag coord con problema: {e}')
#             self.get_logger().debug(traceback.format_exc())
#     def draw_debug_map(self, robot_x, robot_y, raw_u, raw_v):
#         """
#         Dibuja un mapa 2D simulando la mesa vista desde arriba
#         """
#         # 1. Crear lienzo negro (o limpiar el anterior)
#         canvas = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8)

#         # 2. Dibujar Ejes del Robot (Desde el origen visual)
#         # Eje X+ (Hacia la Izquierda fisica)
#         cv2.arrowedLine(canvas, (self.origin_draw_x, self.origin_draw_y), 
#                         (self.origin_draw_x - 100, self.origin_draw_y), (255, 255, 255), 2)
#         cv2.putText(canvas, "X+ (Izq)", (self.origin_draw_x - 140, self.origin_draw_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

#         # Eje Y+ (Hacia Abajo fisico)
#         cv2.arrowedLine(canvas, (self.origin_draw_x, self.origin_draw_y), 
#                         (self.origin_draw_x, self.origin_draw_y + 100), (255, 255, 255), 2)
#         cv2.putText(canvas, "Y+ (Abajo)", (self.origin_draw_x, self.origin_draw_y + 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

#         # 3. Dibujar BASE ROBOT (0,0) - Cuadrado Verde
#         cv2.rectangle(canvas, (self.origin_draw_x - 20, self.origin_draw_y - 20), 
#                       (self.origin_draw_x + 20, self.origin_draw_y + 20), (0, 255, 0), -1)
#         cv2.putText(canvas, "BASE ROBOT", (self.origin_draw_x - 40, self.origin_draw_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

#         # 4. Dibujar CENTRO CAMARA (Offset) - Cruz Amarilla
#         # Recuerda: X crece a la izquierda, Y crece abajo en tu sistema fisico
#         cam_draw_x = int(self.origin_draw_x - self.OFFSET_CAMARA_X)
#         cam_draw_y = int(self.origin_draw_y + self.OFFSET_CAMARA_Y)
        
#         cv2.drawMarker(canvas, (cam_draw_x, cam_draw_y), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
#         cv2.putText(canvas, "CAM CENTER", (cam_draw_x + 10, cam_draw_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

#         # 5. Dibujar LA PAPA (Objetivo) - Circulo Rojo
#         # Transformamos coordenada mm a pixel de dibujo
#         # X robot crece a izq -> restamos de origen
#         target_draw_x = int(self.origin_draw_x - robot_x) 
#         # Y robot crece abajo -> sumamos a origen
#         target_draw_y = int(self.origin_draw_y + robot_y)

#         # Linea desde robot a papa (Azul)
#         cv2.line(canvas, (self.origin_draw_x, self.origin_draw_y), (target_draw_x, target_draw_y), (255, 100, 0), 1)
        
#         # El punto
#         cv2.circle(canvas, (target_draw_x, target_draw_y), 10, (0, 0, 255), -1)
        
#         # Texto de coordenadas
#         text_info = f"Coord: X={robot_x} mm, Y={robot_y} mm"
#         text_pixel = f"Raw Px: ({raw_u:.0f}, {raw_v:.0f})"
#         cv2.putText(canvas, text_info, (target_draw_x + 15, target_draw_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#         cv2.putText(canvas, text_pixel, (target_draw_x + 15, target_draw_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

#         # Mostrar ventana
#         cv2.imshow("Debug Coordenadas Robot", canvas)
#         cv2.waitKey(1)

#     def processing_loop(self):
#         while not self.stop_event.is_set():
#             try:
#                 # obtener data de la cola
#                 data_coord = self.bag_coord_q.get_nowait()
#                 # self.get_logger().info(f'dato_coord es {data_coord}')
#                 # Aqui hay que hacer la transformacion de los pixeles a las coordenadas del robot
#                 x_cam = float(data_coord[0])
#                 y_cam = float(data_coord[1])

#                 x_rel_mm = (self.centro_camara_X - x_cam) * self.factor_resolucion
#                 y_rel_mm = (y_cam - self.centro_camara_Y) * self.factor_resolucion
#                 # trasladar el origen de la esquina a la base del robot
#                 x = int(x_rel_mm + self.OFFSET_CAMARA_X)
#                 y = int(y_rel_mm + self.OFFSET_CAMARA_Y)

#                 # self.get_logger().info(f'Las coordenadas x, y del objeto en las coordenadas del robot es {x, y}')
#                 # Ahora enviar el mensaje, hacer la publicacion
#                 msg = Int16MultiArray()
#                 msg.data = [x, y]
#                 self.publisher.publish(msg)

#                 # 4. DIBUJAR MAPA
#                 # Solo dibujamos si estamos en un entorno gráfico
#                 # try:
#                 #     self.draw_debug_map(x, y, x_cam, y_cam)
#                 # except Exception as e_draw:
#                 #     self.get_logger().warning(f"No se puede dibujar (SSH sin X11?): {e_draw}")


#             except Empty:
#                 continue # completar
#             except Exception as e:
#                 self.get_logger().warning(f'Problemas en el loop de procesamiento: {e}')
#                 self.get_logger().debug(traceback.format_exc())
#                 continue

#     def destroy_threads(self):
#         """
#         Destruir todos los hilos antes de destruir el nodo
#         """
#         self.get_logger().info('Deteniendo hilos de procesamiento')
#         self.stop_event.set()
#         try:
#             self.processing_thread.join(timeout=1.0)
#         except:
#             pass
#         self.get_logger().info('Hilos detenidos')  

# def main(args=None):
#     rclpy.init(args=args)
#     bag_coord_trans_node = BagCoordTransNode()
#     rclpy.spin(bag_coord_trans_node)
#     bag_coord_trans_node.destroy_threads()
#     bag_coord_trans_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
