#!/usr/bin/env python3

"""
Nodo que recibe la imagen de la camara estatica y la procesa para identificar donde se encuentran los eslabones del robot en la imagen.

"""

from http.cookies import _quote
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
from queue import Queue, Empty

class StaticCameraRobotNode(Node):
    def __init__(self):
        super().__init__('static_camera_robot_node')
        self.get_logger().info('[Static Camera Robot Node] ha sido iniciado')

        # Crear cola y elementos de ella
        self.img_q = Queue(maxsize=5) # cola fifo de tamano 5
        self.stop_event = Event()
        # Colas para hilos de procesamiento por color
        self.red_q = Queue(maxsize=2)
        self.blue_q = Queue(maxsize=2)
        self.green_q = Queue(maxsize=2)
        # Suscripcion
        self.subscription = self.create_subscription(Image, 'static_camera/image_raw', self.queue_robot_callback, 10)
        self.get_logger().info('[Static Camera Robot Node]: Suscriptor creado')

        # Bridge
        self.bridge = CvBridge()

        # Resultados aplicacion de mascaras
        self.result_red = None
        self.result_blue = None
        self.result_green = None
        # Mask
        self.red_mask = None
        self.blue_mask = None
        self.green_mask = None

        # Iniciar hilo de procesamiento
        self.processing_thread = Thread(target=self.processing_loop)
        self.processing_thread.start() # inicio el Hilo, no uso un hilo daemon, porque Ros2 ya lo hace internamente
        self.red_thread = Thread(target=self.locate_object, args=(self.red_q,))
        self.red_thread.start()
        self.blue_thread = Thread(target=self.locate_object, args=(self.blue_q,))
        self.blue_thread.start()
        self.green_thread = Thread(target=self.locate_object, args=(self.green_q,))
        self.green_thread.start()
        self.get_logger().info('[Static Camera Robot Node]: Hilo de procesamiento iniciado')


    def queue_robot_callback(self, msg):
        """
        Esta callback recibe el mensaje imagen y debe procesarlo a formato cv2 y ponerlo en la cola si es posible
        """
        try:
            cv2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[Static Camera Robot Node]: Error al convertir la imagen: {e}')
            return
        
        # Ingresar a la cola
        try:
            self.img_q.put_nowait(cv2_image)
        except:
            self.get_logger().warning('La cola de imagenes esta llena, se descarta la imagen actual')
    
    def processing_loop(self):
        """
        Hilo que procesa las imagenes de la cola para detectar los colores de los eslabones

        De momento la idea es usar colores fuertes que no se vean en otros elementos.
        - Rojo
        - Azul
        - Verde
        """
        self.get_logger().info('[Static Camera Robot Node]: Hilo de procesamiento en ejecucion')

        while not self.stop_event.is_set():

            # Obtener imagen de la cola
            try:
                frame = self.img_q.get_nowait()

                frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

                # Definir rangos de color, Rojo, Azul y Verde

                #Rojo
                lower_red = np.array([0, 100, 100])
                upper_red = np.array([1, 255, 255])

                lower_red_2 = np.array([160, 100, 100])
                upper_red_2 = np.array([179, 255, 255])  

                maskr_1 = cv.inRange(frame_hsv, lower_red, upper_red) 
                maskr_2 = cv.inRange(frame_hsv, lower_red_2, upper_red_2)

                self.red_mask = cv.bitwise_or(maskr_1, maskr_2)       

                # Azul
                lower_blue = np.array([100,  70,  70])   # más amplio / suave
                upper_blue = np.array([130, 255, 255])

                self.blue_mask = cv.inRange(frame_hsv, lower_blue, upper_blue)

                # Verde
                lower_green = np.array([45, 85, 85])
                upper_green = np.array([85, 255, 255])

                self.green_mask = cv.inRange(frame_hsv, lower_green, upper_green)

                # Resultados

                self.result_red = cv.bitwise_and(frame, frame, mask = self.red_mask)
                self.result_blue = cv.bitwise_and(frame, frame, mask = self.blue_mask)
                self.result_green = cv.bitwise_and(frame, frame, mask = self.green_mask)

                # Invocar las funciones de localizacion para cada color
                try:
                    self.red_q.put_nowait((self.red_mask.copy(), self.result_red.copy()))
                except:
                    pass # Agregar logger en caso de necesitar debugueo
                try:
                    self.blue_q.put_nowait((self.blue_mask.copy(), self.result_blue.copy()))
                except:
                    pass # Agregar logger en caso de necesitar debugueo
                try:
                    self.green_q.put_nowait((self.green_mask.copy(), self.result_green.copy()))
                except:
                    pass # Agregar logger en caso de necesitar debugueo

                # result = cv.bitwise_or(self.result_red, self.result_blue)
                # result = cv.bitwise_or(result, self.result_green)

                cv.imshow('Static Camera Robot Detection', self.result_green)
                cv.imshow('Rojo', self.result_red)
                cv.imshow('Azul', self.result_blue)
                cv.waitKey(1)

            except Empty:
                frame = None


    def locate_object(self, queue):
        """
        Debe calcular la posicion del objeto de la imagen segmentada
        """
        while not self.stop_event.is_set():
            try:
                mask, result = queue.get(timeout=0.5)
                if mask is None or result is None:
                    continue # En caso de que que alguno sea nulo no tomar en cuenta

                # aplicar transformacion morfologica para unir las 2 segmentaciones, en caso de haber
                kernel = cv.getStructuringElement(cv.MORPH_RECT, (35, 35))
                closed = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

                contours, hierarchy = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # RETR especifica los extremos exteriores del contorno y CHAIN es un metodo de aproximoacion por compresion vertical, horizontal y diagonal
                largest = max(contours, key=cv.contourArea)
                # print(largest)
                # if largest.all() == None:
                #     return None
                # for ct in contours:
                area = cv.contourArea(largest)
                if area < 600:
                    return None, None

                x, y, w, h = cv.boundingRect(largest) # rectangulo
                momento = cv.moments(largest)
                if momento['m00'] != 0:
                    cx = int(momento['m10']/momento['m00'])
                    cy = int(momento['m01']/momento['m00'])

                # cv.drawContours(self.result, [largest], -1, (0,255,0), 2)
                cv.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv.circle(result, (cx,cy), 4, (0,0,255), -1)

                cv.imshow('Find center Contorno', result)
                cv.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"[Find center error]: {e}")
                return False
        return cx, cy



    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('[Static Camera Robot Node]: Deteniendo hilos de procesamiento')
        self.stop_event.set()
        self.processing_thread.join()
        try:
            self.red_thread.join(timeout=1.0)
            self.blue_thread.join(timeout=1.0)
            self.green_thread.join(timeout=1.0)
            cv.destroyAllWindows()
        except:
            pass
        self.get_logger().info('[Static Camera Robot Node]: Hilos detenidos')

def main(args=None):
    rclpy.init(args=args)
    static_camera_robot_node = StaticCameraRobotNode()
    rclpy.spin(static_camera_robot_node)
    static_camera_robot_node.destroy_threads()
    static_camera_robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
