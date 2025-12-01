#!/usr/bin/env python3

"""
Este nodo debe colectar la informacion que venga de los demas nodos y mandarsela al nodo de comunicacion serial
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from threading import Thread, Event, Condition
from queue import Queue, Empty, Full
import traceback
from time import sleep

class ArduinoCoordPubNode(Node):
    def __init__(self):
        super().__init__('arduino_coord_pub_node')

        self.get_logger().info('[Arduino Coord Pub Node]: ha sido iniciado')


        # Atributos
        # self.emergency = 0 # Si este atributo se vuelve 1, mandar mensaje de emergencia al comunicador serial inmediatamente

        # colas y condiciones
        # self.kinematics_q = Queue(maxsize=1)
        # self.orientation_q = Queue(maxsize=1)
        self._cv = Condition()
        self.last_kin = None
        self.last_ori = None
        self.stop_event = Event()
        # Hilos
        self.processing_thread = Thread(target=self.processing_loop, daemon=False)
        self.processing_thread.start()

        # Lista de mensajes de prueba
        # self.test_msg_serial_list = ['0', '1', 'holaaa', '2324', '67', '1726861387', '333333', '29387928', 'aaaaa', 'kjcbhawkuckbshdc', '1234567890987654321', 'qwertyuioolkjhgfdsazxcvbn']
        # Publicadores y Subscriptores

        self.pub = self.create_publisher(String, 'arduino/command/coord', 10)
        self.emergency_pub = self.create_publisher(String, 'arduino/command/emergency', 10)

        self.emergency_sub = self.create_subscription(Int16MultiArray, 'emergency/msg', self.emergency_callback, 10) # mensaje del paro de emergencia de software

        self.kinematics_sub = self.create_subscription(Int16MultiArray, 'inv_kinematics/angles', self.kinematics_callback, 10) # mensaje de la cinematica


        self.orientation_sub = self.create_subscription(Int16MultiArray, 'orientation/papa_orientation', self.orientation_callback, 10)


        # self.input_thread =  Thread(target=self.get_coord, daemon=True)
        # self.input_thread.start()
    
    def emergency_callback(self, msg):
        try:
            self.get_logger().info(f'El largo del mensaje es {len(msg.data)} y el mensaje {msg}')
            if msg.data is not None:
                data = list(msg.data)
                self.get_logger().info(f' esta llegando este mensaje {data}, msg {msg}')
                if data[0] == 1:
                # estamos en una emergencia, de momento si se para, se para el sistema completo y hay que reiniciar el sistema completo
                    emergency_msg = String()
                    emergency_msg.data = 'Stop;M1:XX;M2:XX;S:XX;XXX' 
                    self.emergency_pub.publish(emergency_msg)
                    self.get_logger().warning(f'Se ha activado un paro de emergencia, enviando al nodo serial la orden, reiniciar sistema')
                    self.stop_event.set() #Paralizo los hilos, hay que reiniciar el sistema
            else:
                return
        except Exception as e:
            self.get_logger().warning(f'Error al enviar senal de emergencia: {e}')
            return

    def kinematics_callback(self, msg):
        try:
            data = list(msg.data)
            self.get_logger().info(f' El mensaje que llego a la cinematica es: {data}')
        #     self.kinematics_q.put_nowait()
        # except Full:
        #     self.get_logger().warning(f'La cola de las coordenadas de cinematica esta llena')
            with self._cv:
                self.last_kin = data
                self._cv.notify_all()
        except Exception as e:
            self.get_logger().warning(f'Cola de cinematica con problema: {e}')
            self.get_logger().debug(traceback.format_exc())  

    def orientation_callback(self, msg):
        try:
            data = list(msg.data)
            self.get_logger().info(f' El mensaje que llego a la orientacion es: {data}')
        #     self.orientation_q.put_nowait()
        # except Full:
        #     self.get_logger().warning(f'La cola de orientacion esta llena')
            with self._cv:
                self.last_ori = data
                self._cv.notify_all()        
        except Exception as e:
            self.get_logger().warning(f'Cola de orientacion con problema: {e}')
            self.get_logger().debug(traceback.format_exc())  

    def processing_loop(self):
        while not self.stop_event.is_set():
            with self._cv:
                # esperar a las 2 coordenadas o al stop_event
                ambos = self._cv.wait_for(lambda:(self.last_kin is not None and self.last_ori is not None) or self.stop_event.is_set(), timeout=0.5)
                if self.stop_event.is_set():
                    break
                if not ambos:
                    continue
                kin = self.last_kin
                ori = self.last_ori

                self.last_kin = None
                self.last_ori = None
                # self.get_logger().info(f'Loop de procesamiento, las coordenadas que hay son {kin}, {ori}')

            # Procesar para el mensaje
            try:
                if kin is not None and ori is not None:
                    # Angulo de la cinematica
                    theta_1 = kin[0]
                    theta_2 = kin[1]
                    gamma = ori[0]
                    # signos
                    if theta_1 >= 0:
                        s_theta_1 = 1
                    else: 
                        s_theta_1 = 0

                    if theta_2 >= 0:
                        s_theta_2 = 1
                    else: 
                        s_theta_2 = 0

                    if gamma >= 0:
                        s_gamma = 1
                    else: 
                        s_gamma = 0
                    coord_msg = String()
                    coord_msg.data = f'Mover;M1:{abs(theta_1)};M2:{abs(theta_2)};S:{abs(gamma)};{s_theta_1}{s_theta_2}{s_gamma}'
                    self.pub.publish(coord_msg)
                    self.get_logger().info(f'Enviando comando al comunicador serial')
            except Exception as e:
                self.get_logger().warning(f'Error al enviar mensaje de cinematica: {e}')
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

    # def get_coord(self):
    #     """
    #     De momento pide un input para mandar por consola una coordenada
    #     """ 
    #     # Contador para testeo de mensaje
    #     ct = 0
    #     while rclpy.ok():
    #         msg = String()
    #         msg.data = self.test_msg_serial_list[ct]
    #         self.pub.publish(msg)

    #         ct +=1
    #         if ct >= len(self.test_msg_serial_list):
    #             ct = 0

    #         #!TODO Nota para el yo de mañana que cansado se olvidara de lo que hizo
    #         """
    #         Recibir los mensajes por el input de consola es un cacho, seria mejor que para una demostracion, hacer una rutina de coordenadas en una lista y recorrerla mientras se envian, ambos nodos deberian funcionar a traves de un run nodo y no con el launch
            # """


def main(args=None):
    rclpy.init(args=args)
    arduino_coord_pub = ArduinoCoordPubNode()
    rclpy.spin(arduino_coord_pub)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
