#!/usr/bin/env python3

"""
Nodo que recibe la imagen de la camara estatica y la procesa para identificar donde se encuentran los eslabones del robot en la imagen.

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

class StaticCameraRobotNode(Node):
    def __init__(self):
        super().__init__('static_camera_robot_node')
        self.get_logger().info('[Static Camera Robot Node] ha sido iniciado')

        # Crear cola y elementos de ella
        self.img_q = Queue(maxsize=5) # cola fifo de tamano 5
        self.stop_event = Event()
        # Colas para hilos de procesamiento por color
        self.red_q = Queue(maxsize=5)
        # self.blue_q = Queue(maxsize=5)
        # self.orange_q = Queue(maxsize=5) # descomentar en caso de usar el naranjo
        self.green_q = Queue(maxsize=5)
        # Suscripcion
        self.subscription = self.create_subscription(Image, 'static_camera/image_raw', self.queue_robot_callback, 10)
        self.get_logger().info('[Static Camera Robot Node]: Suscriptor creado')
        # Publicador
        self.min_bbox_publisher = self.create_publisher(Int16MultiArray, 'static_camera_robot/min_bbox_coord', 10) # Este publicado debe mandar las coordenadas de los bbox de los 3 colores.
        self._destroyed = False

        # Bridge
        self.bridge = CvBridge()

        # Resultados aplicacion de mascaras
        self.result_red = None
        # self.result_blue = None
        # self.result_orange = None
        self.result_green = None
        # Mask
        self.red_mask = None
        # self.blue_mask = None
        # self.orange_mask = None
        self.green_mask = None

        # Color coord
        self.red_coord = None
        # self.blue_coord = None
        # self.orange_coord = None
        self.green_coord = None

        # bbox coord
        self.red_bbox = None
        # self.blue_coord = None
        # self.orange_bbox = None
        self.green_bbox = None


        # Iniciar hilo de procesamiento
        self.processing_thread = Thread(target=self.processing_loop)
        self.processing_thread.start() # inicio el Hilo, no uso un hilo daemon, porque Ros2 ya lo hace internamente
        self.red_thread = Thread(target=self.locate_object, args=(self.red_q, 'red_coord', 'red_bbox'))
        self.red_thread.start()
        # self.blue_thread = Thread(target=self.locate_object, args=(self.blue_q, 'blue_coord'))
        # self.blue_thread.start()
        # Descomentar en caso de usar el naranjo
        # self.orange_thread = Thread(target= self.locate_object, args=(self.orange_q, 'orange_coord', 'orange_bbox'))
        # self.orange_thread.start()
        self.green_thread = Thread(target=self.locate_object, args=(self.green_q, 'green_coord', 'green_bbox'))
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
                # lower_blue = np.array([100,  70,  70])   # más amplio / suave
                # upper_blue = np.array([130, 255, 255])

                # self.blue_mask = cv.inRange(frame_hsv, lower_blue, upper_blue)
                # Naranjo
                # Se agrega el naranjo y se cambia por el azul ya que la manguera de la bomba es azul
                # DESCOMENTAR EN CASO QUE SE USE EL NARANJO
                # lower_orange = np.array([12,  120,  120])
                # upper_orange = np.array([20, 255, 255])

                # self.orange_mask = cv.inRange(frame_hsv, lower_orange, upper_orange)


                # Verde
                # lower_green = np.array([45, 85, 85])
                # upper_green = np.array([85, 255, 255])
                
                
                # ! Umbrales de testeo fuera del lab
                lower_green = np.array([60, 85, 85])
                upper_green = np.array([80, 255, 255])

                self.green_mask = cv.inRange(frame_hsv, lower_green, upper_green)

                # Resultados

                self.result_red = cv.bitwise_and(frame, frame, mask = self.red_mask)
                # self.result_blue = cv.bitwise_and(frame, frame, mask = self.blue_mask)
                # self.result_orange = cv.bitwise_and(frame, frame, mask = self.orange_mask)
                self.result_green = cv.bitwise_and(frame, frame, mask = self.green_mask)

                # Invocar las funciones de localizacion para cada color
                try:
                    self.red_q.put_nowait((self.red_mask.copy(), self.result_red.copy()))
                except Full:
                    self.get_logger().warning('red_q llena — descartando frame')
                except Exception as e:
                    self.get_logger().error(f'Error al ingresar en cola roja: {e}')
                     # Agregar logger en caso de necesitar debugueo
                # try:
                #     self.blue_q.put_nowait((self.blue_mask.copy(), self.result_blue.copy()))
                # except Full:
                #     self.get_logger().warning('red_q llena — descartando frame')                
                # except Exception as e:
                #     self.get_logger().error(f'Error al ingresar en cola azul: {e}')

                #DESCOMENTAR EN CASO QUE SE USE EL NARANJO
                # try:
                #     self.orange_q.put_nowait((self.orange_mask.copy(), self.result_orange.copy()))
                # except Full:
                #     self.get_logger().warning('orange_q llena — descartando frame')
                # except Exception as e:
                #     self.get_logger().error(f'Error al ingresar en cola naranja: {e}')

                try:
                    self.green_q.put_nowait((self.green_mask.copy(), self.result_green.copy()))
                except Full:
                    self.get_logger().warning('green_q llena — descartando frame')
                except Exception as e:
                    self.get_logger().error(f'Error al ingresar en cola verde: {e}')

                # result = cv.bitwise_or(self.result_red, self.result_blue)
                # result = cv.bitwise_or(result, self.result_green)

                # cv.imshow('Static Camera Robot Detection', self.result_green)
                # cv.imshow('Rojo', self.result_red)
                # cv.imshow('naranjo', self.result_orange)
                # cv.waitKey(1)
                # self.get_logger().info(f' Centro rojo: {self.red_coord}')
                # # self.get_logger().info(f' Centro azul: {self.blue_coord}')
                # self.get_logger().info(f'Centro naranja: {self.orange_coord}')
                # self.get_logger().info(f' Centro verde: {self.green_coord}')

                # Publicar las coordenadas obtenidas, solo si los 3 colores se pueden ver
                try: # TODO el naranjo se elimino poruqe no sera necesario, pero el codigo se mantiene por si acaso
                    # self.get_logger().info(f'las coordenadas estan llegando de esta manera  {self.orange_bbox}, {self.green_bbox}, {self.red_bbox},')

                    # En caso de usar el naranjo descomentar esto:
                    # if self.red_bbox is not None and self.orange_bbox is not None and self.green_bbox is not None:

                    #     bbox_cord = np.array([self.orange_bbox, self.green_bbox, self.red_bbox])
                    if self.red_bbox is not None and self.green_bbox is not None:

                        bbox_cord = np.array([self.green_bbox, self.red_bbox])
                        bbox_cord = [int(x) for x in bbox_cord.flatten().tolist()]


                        if rclpy.ok() and not getattr(self, '_destroyed', False):
                            msg = Int16MultiArray()
                            msg.data = bbox_cord
                            try:
                                self.min_bbox_publisher.publish(msg)
                                # self.get_logger().info('Enviando bbox colores')
                            except Exception as e:
                                self.get_logger().warning(f'Problema al publicar el mensaje: {e}')
                                continue
                except Exception as e:
                    self.get_logger().warning(f'Error al enviar bbox de colores {e}')
                    continue
                    

            except Empty:
                frame = None
                continue


    def locate_object(self, queue, coord, bbox):
        """
        Debe calcular la posicion del objeto de la imagen segmentada
        """
        while not self.stop_event.is_set():
            try:
                mask, result = queue.get(timeout=0.5)
                if mask is None or result is None:
                    self.get_logger().info(f'No esta recibiendo mascara o imagen {mask}, {result}')
                    setattr(self, coord, None)
                    continue # En caso de que que alguno sea nulo no tomar en cuenta

                # aplicar transformacion morfologica para unir las 2 segmentaciones, en caso de haber
                kernel = cv.getStructuringElement(cv.MORPH_RECT, (35, 35))
                closed = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

                contours, hierarchy = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # RETR especifica los extremos exteriores del contorno y CHAIN es un metodo de aproximoacion por compresion vertical, horizontal y diagonal
                if not contours:
                    setattr(self, coord, None)
                    setattr(self, bbox, None)
                    continue
                largest = max(contours, key=cv.contourArea)
                # print(largest)
                # if largest.all() == None:
                #     return None
                # for ct in contours:
                area = cv.contourArea(largest)
                if area < 600:
                    setattr(self, coord, None)
                    setattr(self, bbox, None)
                    continue

                # x, y, w, h = cv.boundingRect(largest) # rectangulo
                rect = cv.minAreaRect(largest)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                momento = cv.moments(largest)
                if momento['m00'] != 0:
                    cx = int(momento['m10']/momento['m00'])
                    cy = int(momento['m01']/momento['m00'])
                    setattr(self, coord, (cx, cy))
                    setattr(self, bbox, box)

                # cv.drawContours(self.result, [largest], -1, (0,255,0), 2)
                # cv.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv.drawContours(result,[box],0,(0,0,255),2)
                cv.circle(result, (cx,cy), 4, (0,0,255), -1)

                cv.imshow('Find center Contorno Robot', result)
                cv.waitKey(1)
            except Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"[Find center error robot color]: {e}")
                self.get_logger().debug(traceback.format_exc())
                continue
        # return cx, cy



    def destroy_threads(self):
        """
        Destruir todos los hilos antes de destruir el nodo
        """
        self.get_logger().info('[Static Camera Robot Node]: Deteniendo hilos de procesamiento')
        self.stop_event.set()
        self._destroyed = True
        self.processing_thread.join()
        try:
            self.red_thread.join(timeout=1.0)
            # self.blue_thread.join(timeout=1.0)
            # self.orange_thread.join(timeout=1.0)
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
