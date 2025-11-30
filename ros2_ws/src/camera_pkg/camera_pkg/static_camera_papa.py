#!/usr/bin/env python3

"""
Nodo que recibe la imagen de la camara estatica y la procesa para identificar donde se encuentra la bolsa de papa en la imagen.

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

class StaticCameraPapaNode(Node):
    def __init__(self):
        super().__init__('static_camera_papa_node')

        self.get_logger().info('[Static Camera Papa Node]: ha sido iniciado')

        # Crear cola para pasar imagenes del callback al hilo de procesamiento
        self.img_q = Queue(maxsize=5) # Cola Fifo de tamaño 5
        self.stop_event = Event()

        # Crear suscriptor al mensaje de la camara estatica
        self.subscription = self.create_subscription(Image, 'static_camera/image_raw', self.queue_papa_callback, 10)
        self.get_logger().info('[Static Camera Papa Node]: Suscriptor creado')

        # Crear publicador
        self.publisher = self.create_publisher(Int16MultiArray, 'static_camera/papa_coord', 10) # Publica las coordenadas en formato int de 16 bits

        self.publisher_bbox = self.create_publisher(Int16MultiArray, 'static_camera/min_bbox', 10) # Publica las coordenadas del min bounding box

        # atributo de resultado
        self.result = None

        # Bridge
        self.bridge = CvBridge()

        # Iniciar hilo de procesamiento
        self.processing_thread = Thread(target=self.processing_loop)
        self.processing_thread.start() # inicio el Hilo, no uso un hilo daemon, porque Ros2 ya lo hace internamente
        self.get_logger().info('[Static Camera Papa Node]: Hilo de procesamiento iniciado')

    def queue_papa_callback(self, msg):
        """
        Esta callback recibe el mensaje imagen y debe procesarlo a formato cv2 y ponerlo en la cola si es posible
        """
        try:
            cv2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[Static Camera Papa Node]: Error al convertir la imagen: {e}')
            return
        
        # Ingresar a la cola
        try:
            self.img_q.put_nowait(cv2_image)
        except Full:
            self.get_logger().warning('La cola de imagenes esta llena, se descarta la imagen actual')
        except Exception as e:
            self.get_logger().warning(f'Error al encolar imagen: {e}')
            self.get_logger().debug(traceback.format_exc())
    
    def processing_loop(self):
        """
        Hilo que procesa las imagenes de la cola para detectar la bolsa de papa y ubicar las coordenadas de su centro
        """
        self.get_logger().info('[Static Camera Papa Node]: Hilo de procesamiento en ejecucion')

        while not self.stop_event.is_set():
                
            # Obtener la imagen de la cola
            try:
                # frame = self.img_q.get_nowait()
                frame = self.img_q.get(timeout=0.5)
                # Pasar la imagen al espacio de color HSV
                frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 

                # Definir rangos de color para detectar la bolsa de papa (amarillo), parametros ajustados a la iluminacion del lab

                # lower_yellow = np.array([25, 125, 125])
                # upper_yellow = np.array([55, 235, 235]) #H, S, V

                # Rangos para testear fuera del lab
                lower_yellow = np.array([25, 135, 140])
                upper_yellow = np.array([55, 235, 235])

                #Rojo
                # lower_red = np.array([0, 100, 100])
                # upper_red = np.array([1, 255, 255])

                # lower_red_2 = np.array([160, 100, 100])
                # upper_red_2 = np.array([179, 255, 255])

                # # Mascara Roja
                # mask_1 = cv.inRange(frame_hsv, lower_red, upper_red) # imagen, rango bajo, rango alto
                # mask_2 = cv.inRange(frame_hsv, lower_red_2, upper_red_2)

                # mask = cv.bitwise_or(mask_1, mask_2)

                # Mascara amarilla
                yellow_mask = cv.inRange(frame_hsv, lower_yellow, upper_yellow)
                kernel = cv.getStructuringElement(cv.MORPH_RECT, (35, 35))
                yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_CLOSE, kernel)


                # Aplicar la mascara a la imagen original
                self.result = cv.bitwise_and(frame, frame, mask=yellow_mask)
                self.find_center_papa(yellow_mask)
                # self.get_logger().info(f"El centro horizontal del objeto se encuentra en: {center}")

                # Mostrar imagen para testeo
                # cv.imshow('Static Camera Papa Detection', self.result)
                # cv.waitKey(1)

            except Empty:
                frame = None
                continue
                # self.get_logger().info('[Static Camera Papa Node]: La cola de imagenes esta vacia, esperando nueva imagen')
                # Al parecer la cola avanza muy rapido y este mensaje satura la terminal, lo comentare mientras tanto

        
    def find_center_papa(self, mask):
        """
        Encuentra el el centro de masa del objeto (dibujo un rectangulo alrededor de el) y retorna las coordenadas dentro del frame, es decir de la matriz que corresponde a la imagen, de la bolsa de papas
        """
        # La mascara ya es binaria, por lo cual la recibo
        ### Metodo nuevo con funciones de opencv
        try:
            copy_mask = mask.copy() 

            # aplicar transformacion morfologica para unir las 2 segmentaciones, en caso de haber
            kernel = cv.getStructuringElement(cv.MORPH_RECT, (35, 35))
            closed = cv.morphologyEx(copy_mask, cv.MORPH_CLOSE, kernel)

            contours, hierarchy = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # RETR especifica los extremos exteriores del contorno y CHAIN es un metodo de aproximoacion por compresion vertical, horizontal y diagonal
            largest = max(contours, key=cv.contourArea)
            # print(largest)
            # if largest.all() == None:
            #     return None
            # for ct in contours:
            area = cv.contourArea(largest)
            if area < 600:
                return None, None

            # x, y, w, h = cv.boundingRect(largest) # rectangulo

            rect = cv.minAreaRect(largest) # minimo rectangulo
            box = cv.boxPoints(rect)
            box = np.int0(box)
            # print(box)
            momento = cv.moments(largest)

            if momento['m00'] != 0:
                cx = int(momento['m10']/momento['m00'])
                cy = int(momento['m01']/momento['m00'])

            # cv.drawContours(self.result, [largest], -1, (0,255,0), 2)
            cv.drawContours(self.result,[box],0,(0,0,255),2) # minimo rectangulo
            # cv.rectangle(self.result, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv.circle(self.result, (cx,cy), 4, (0,0,255), -1)

            # Publicar mensaje de la min bbox
            msg = Int16MultiArray()
            bbox = [int(x) for x in box.flatten().tolist()]
            msg.data = bbox
            self.publisher_bbox.publish(msg)
            # self.get_logger().info(f'Enviando mensaje de la bbox de la bolsa')
                # Publicar el mensaje con lsa coordenadas del respectivo pixel del centro
            try:
                msg = Int16MultiArray()
                msg.data = [int(cx), int(cy)]
                self.publisher.publish(msg)
            except Exception as e:
                # self.get_logger().warning(f"[Static camera papa publisher] No se ha detectado un objeto, no se publican coordenadas")
                pass
            cv.imshow('Find center Contorno Papa', self.result)
            cv.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[Find center error papa]: {e}")
            return False
        # return cx, cy
        ## Testear y mejorar en laboratorio


    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('[Static Camera Papa Node]: Deteniendo hilos de procesamiento')
        self.stop_event.set()
        self.processing_thread.join()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        self.get_logger().info('[Static Camera Papa Node]: Hilos detenidos')

def main(args=None):
    rclpy.init(args=args)
    static_camera_papa_node = StaticCameraPapaNode()
    rclpy.spin(static_camera_papa_node)
    static_camera_papa_node.destroy_threads()
    static_camera_papa_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
