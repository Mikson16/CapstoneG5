#!/usr/bin/env python3

#se debe activar sudo pigpio cada vez que se active el programa para habilitar la comunicacion
"""
Este nodo debe levantar y escuchar los pines GPIO de la raspi que conectan con las salidas A, B de los encoders de los Pololu, luego envia esa informacion (es posible que deba procesarlos este mismo nodo o enviarlos a procesar a otro lado)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from threading import Thread, Event
from time import sleep
import pigpio

class RaspiGpioReaderNode(Node):
    pass
