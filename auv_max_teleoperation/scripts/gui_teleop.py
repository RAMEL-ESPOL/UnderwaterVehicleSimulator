#!/usr/bin/env python3

import tkinter as tk
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# Definición de los límites de velocidad como en el código C++
LIMIT_VEL_LZ = 2.75
LIMIT_VEL_AY = 0.25
LIMIT_VEL_LX = 0.75
LIMIT_VEL_AZ = 1.75

# Clase para el nodo de teleoperación
class MaxTeleopNode(Node):
    def __init__(self):
        super().__init__('auv_teleop_gui')
        self.publisher_ = self.create_publisher(Twist, '/model/auv_max/cmd_vel', 10)
        self.cmd_vel_msg = Twist()

        self.startVel = False

    def publish_velocity(self):
        if self.startVel:
            # Limitar las velocidades antes de publicar
            self.cmd_vel_msg.linear.x = max(min(self.cmd_vel_msg.linear.x, LIMIT_VEL_LX), -LIMIT_VEL_LX)
            self.cmd_vel_msg.linear.z = max(min(self.cmd_vel_msg.linear.z, LIMIT_VEL_LZ), -LIMIT_VEL_LZ)
            self.cmd_vel_msg.angular.y = max(min(self.cmd_vel_msg.angular.y, LIMIT_VEL_AY), -LIMIT_VEL_AY)
            self.cmd_vel_msg.angular.z = max(min(self.cmd_vel_msg.angular.z, LIMIT_VEL_AZ), -LIMIT_VEL_AZ)
            self.publisher_.publish(self.cmd_vel_msg)

    def update_velocity(self, linear_x=None, linear_z=None, angular_y=None, angular_z=None):
        if linear_x is not None:
            self.cmd_vel_msg.linear.x = linear_x
        if linear_z is not None:
            self.cmd_vel_msg.linear.z = linear_z
        if angular_y is not None:
            self.cmd_vel_msg.angular.y = angular_y
        if angular_z is not None:
            self.cmd_vel_msg.angular.z = angular_z
        self.publish_velocity()

def startTeleop(nodeTeleop):
    nodeTeleop.startVel = True

def stopAll(nodeTeleop):
    slider_vyaw.set(0)
    slider_vpitch.set(0)
    slider_vx.set(0)
    slider_vz.set(0)

    nodeTeleop.update_velocity(0.0, 0.0, 0.0, 0.0)
    nodeTeleop.startVel = False

def emergencia(nodeTeleop):
    nodeTeleop.startVel = True
    
    slider_vyaw.set(0)
    slider_vpitch.set(0)
    slider_vx.set(0)
    slider_vz.set(LIMIT_VEL_LZ)

    nodeTeleop.update_velocity(0.0, LIMIT_VEL_LZ, 0.0, 0.0)

# Iniciar ROS2
rclpy.init(args=None)
node = MaxTeleopNode()

# Crear la ventana de Tkinter
root = tk.Tk()
root.title("Control de Teleoperación AUV Max")

# Controles deslizantes para la velocidad
slider_vx = tk.Scale(root, from_=-LIMIT_VEL_LX, to=LIMIT_VEL_LX, resolution=0.01, orient='vertical', label='Velocidad X')
slider_vx.set(0)  # Inicializar con velocidad 0
slider_vx.pack(side='left')

slider_vz = tk.Scale(root, from_=-LIMIT_VEL_LZ, to=LIMIT_VEL_LZ, resolution=0.01, orient='vertical', label='Velocidad Z')
slider_vz.set(0)
slider_vz.pack(side='right')

slider_vyaw = tk.Scale(root, from_=-LIMIT_VEL_AZ, to=LIMIT_VEL_AZ, resolution=0.01, orient='horizontal', label='Velocidad Angular Yaw')
slider_vyaw.set(0)
slider_vyaw.pack(side='right')

slider_vpitch = tk.Scale(root, from_=-LIMIT_VEL_AY, to=LIMIT_VEL_AY, resolution=0.01, orient='horizontal', label='Velocidad Angular Pitch')
slider_vpitch.set(0)
slider_vpitch.pack(side='left')

# Actualización de las velocidades al mover los controles deslizantes
slider_vx.bind("<Motion>", lambda event: node.update_velocity(linear_x=slider_vx.get()))
slider_vz.bind("<Motion>", lambda event: node.update_velocity(linear_z=slider_vz.get()))
slider_vyaw.bind("<Motion>", lambda event: node.update_velocity(angular_z=slider_vyaw.get()))
slider_vpitch.bind("<Motion>", lambda event: node.update_velocity(angular_y=slider_vpitch.get()))

# Botones de control
btn_start = tk.Button(root, text="Iniciar", command=lambda: startTeleop(node))
btn_start.pack()

btn_stop = tk.Button(root, text="Detener", command=lambda: stopAll(node))
btn_stop.pack()

btn_emergency = tk.Button(root, text="Emergencia", command=lambda: emergencia(node))
btn_emergency.pack()

# Ejecutar la aplicación Tkinter
root.mainloop()

# Limpiar y cerrar ROS2
node.destroy_node()
rclpy.shutdown()
