#!/usr/bin/env python3

import tkinter as tk
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# Clase para el nodo de teleoperación
class MaxTeleopNode(Node):
    def __init__(self):
        super().__init__('auv_teleop_gui')
        self.publisher_ = self.create_publisher(Twist, '/model/auv_max/cmd_vel', 10)
        self.cmd_vel_msg = Twist()

    def publish_velocity(self):
        self.publisher_.publish(self.cmd_vel_msg)

    def set_velocity(self, linear_x, linear_z, angular_y, angular_z):
        self.cmd_vel_msg.linear.x = linear_x
        self.cmd_vel_msg.linear.z = linear_z
        self.cmd_vel_msg.angular.y = angular_y
        self.cmd_vel_msg.angular.z = angular_z
        self.publish_velocity()

# Función para iniciar la teleoperación
def iniciar_teleop(node):
    print("Teleoperación iniciada.")

# Función para parar los propulsores
def stop_propulsors(node):
    node.set_velocity(0.0, 0.0, 0.0, 0.0)

# Función para el paro de emergencia
def emergency_stop(node):
    # Establecer la velocidad vertical máxima para emergencia
    node.set_velocity(0.0, 2.75, 0.0, 0.0)

# Iniciar ROS2
rclpy.init(args=None)
node = MaxTeleopNode()

# Crear la ventana de Tkinter
root = tk.Tk()
root.title("Control de Teleoperación AUV Max")

# Crear los controles de teleoperación
slider_vz = tk.Scale(root, from_=-100, to=100, orient='vertical', label='Vz')
slider_vz.pack(side='right')

slider_vyaw = tk.Scale(root, from_=-100, to=100, orient='horizontal', label='Vyaw')
slider_vyaw.pack(side='left')

slider_vpitch = tk.Scale(root, from_=-100, to=100, orient='horizontal', label='Vpitch')
slider_vpitch.pack(side='right')

# Botones de control
btn_iniciar = tk.Button(root, text="Iniciar", command=lambda: iniciar_teleop(node))
btn_iniciar.pack()

btn_emergencia = tk.Button(root, text="Paro Emergencia", command=lambda: emergency_stop(node))
btn_emergencia.pack()

btn_detener = tk.Button(root, text="Detener", command=lambda: stop_propulsors(node))
btn_detener.pack()

# Ejecutar la aplicación Tkinter
root.mainloop()

# Limpiar y cerrar ROS2
node.destroy_node()
rclpy.shutdown()
