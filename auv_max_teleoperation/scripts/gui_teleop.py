#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, N, W, E, S
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from PIL import Image as PilImage
from PIL import ImageTk
import cv2
import threading

# Definición de los límites de velocidad 
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

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/model/auv_max/camera',
            self.listener_callback,
            10)
        self.cv_bridge = CvBridge()
        self.image = None

        self.subscription_odom = self.create_subscription(
            Odometry,
            'model/auv_max/odometry',
            self.odom_callback,
            10)

        self.pos_x = None
        self.pos_y = None
        self.pos_z = None
        self.yaw = None
        self.pitch = None
        self.roll = None

    def listener_callback(self, data):
        cv_image = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        scale = 0.4
        new_width = int(cv_image.shape[1] * scale)
        new_height = int(cv_image.shape[0] * scale)
        new_size = (new_width, new_height)

        cv_image_resized = cv2.resize(cv_image, new_size, interpolation=cv2.INTER_AREA)

        cv2.putText(cv_image_resized, f'Pos X: {self.pos_x} m', (3, new_height-40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,0,0), 1)
        cv2.putText(cv_image_resized, f'Pos Y: {self.pos_y} m', (3, new_height-25), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,0,0), 1)
        cv2.putText(cv_image_resized, f'Pos Z: {self.pos_z} m', (3, new_height-10), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,0,0), 1)

        cv2.putText(cv_image_resized, f'Roll: {self.roll} rad', (new_width-145, new_height-40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,0,0), 1)
        cv2.putText(cv_image_resized, f'Pitch: {self.pitch} rad', (new_width-145, new_height-25), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,0,0), 1)
        cv2.putText(cv_image_resized, f'Yaw: {self.yaw} rad', (new_width-145, new_height-10), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,0,0), 1)

        self.image = cv2.cvtColor(cv_image_resized, cv2.COLOR_BGR2RGB)
    
    def odom_callback(self, data):
        self.pos_x = round(data.pose.pose.position.x, 2)
        self.pos_y = round(data.pose.pose.position.y, 2)
        self.pos_z = round(data.pose.pose.position.z, 2)
        self.yaw = round(data.pose.pose.orientation.z, 2)
        self.pitch = round(data.pose.pose.orientation.y, 2)
        self.roll = round(data.pose.pose.orientation.x, 2)

def update_image_label(image_label, node):
    if node.image is not None:
        pil_image = PilImage.fromarray(node.image)
        tk_image = ImageTk.PhotoImage(image=pil_image)
        image_label.configure(image=tk_image)
        image_label.image = tk_image  # evitar que la imagen sea recolectada por el recolector de basura
    root.after(50, update_image_label, image_label, node)

def toggle_controls(state):
    slider_vx.config(state=state)
    slider_vz.config(state=state)
    slider_vyaw.config(state=state)
    slider_vpitch.config(state=state)
    btn_stop.config(state=state)
    btn_shutdown.config(state=state)
    btn_emergency.config(state=state)

    btn_shutdown.config(bg='#e07576' if state == 'disabled' else '#CB191A')
    btn_stop.config(bg='#e07576' if state == 'disabled' else '#CB191A')
    btn_emergency.config(bg='#e2e584' if state == 'disabled' else '#C5CB09')

def set_val_sliders(value_slider_vx=0, value_slider_vz=0, value_slider_vyaw=0, value_slider_vpitch=0):
    slider_vx.set(value_slider_vx)
    slider_vz.set(value_slider_vz)
    slider_vyaw.set(value_slider_vyaw)
    slider_vpitch.set(value_slider_vpitch)

def start_teleop(nodeTeleop):
    nodeTeleop.startVel = True
    toggle_controls('normal')

def shutdown_teleop(nodeTeleop):
    set_val_sliders(0, 0, 0, 0)

    nodeTeleop.update_velocity(0.0, 0.0, 0.0, 0.0)
    nodeTeleop.startVel = False
    toggle_controls('disabled')

def stop_all(nodeTeleop):
    set_val_sliders(0, 0, 0, 0)

    nodeTeleop.update_velocity(0.0, 0.0, 0.0, 0.0)

def emergencia(nodeTeleop):
    nodeTeleop.startVel = True

    set_val_sliders(value_slider_vz=LIMIT_VEL_LZ)

    nodeTeleop.update_velocity(0.0, LIMIT_VEL_LZ, 0.0, 0.0)

# Iniciar ROS2
rclpy.init(args=None)
image_node = ImageSubscriber()
teleop_node = MaxTeleopNode()

# Crear la ventana de Tkinter
root = tk.Tk()
root.title("Control de Teleoperación AUV Max")

mainframe = tk.Frame(root, bg='#0E111C')

slider_vx = tk.Scale(mainframe, from_=LIMIT_VEL_LX, to=-LIMIT_VEL_LX, resolution=0.01, orient='vertical', label='Vel X', length=175, bg='#0E111C', fg='#e7eaf3')
slider_vz = tk.Scale(mainframe, from_=LIMIT_VEL_LZ, to=-LIMIT_VEL_LZ, resolution=0.01, orient='vertical', label='Vel Z', length=175, bg='#0E111C', fg='#e7eaf3')
slider_vyaw = tk.Scale(mainframe, from_=-LIMIT_VEL_AZ, to=LIMIT_VEL_AZ, resolution=0.01, orient='horizontal', label='Vel Yaw', length=125, bg='#0E111C', fg='#e7eaf3')
slider_vpitch = tk.Scale(mainframe, from_=-LIMIT_VEL_AY, to=LIMIT_VEL_AY, resolution=0.01, orient='horizontal', label='Vel Pitch', length=125, bg='#0E111C', fg='#e7eaf3')

set_val_sliders(0, 0, 0, 0)

# Actualización de las velocidades al mover los controles deslizantes
slider_vx.bind("<Motion>", lambda event: teleop_node.update_velocity(linear_x=slider_vx.get()))
slider_vz.bind("<Motion>", lambda event: teleop_node.update_velocity(linear_z=slider_vz.get()))
slider_vyaw.bind("<Motion>", lambda event: teleop_node.update_velocity(angular_z=slider_vyaw.get()))
slider_vpitch.bind("<Motion>", lambda event: teleop_node.update_velocity(angular_y=slider_vpitch.get()))

# Botones de control
btn_start = tk.Button(mainframe, text="Start", command=lambda: start_teleop(teleop_node), width=14, height=1, bd=4, bg='#358749', activebackground='#23AF46')
btn_shutdown = tk.Button(mainframe, text="Shutdown", command=lambda: shutdown_teleop(teleop_node), width=14, height=1, bd=4, activebackground='#E83637')
btn_stop = tk.Button(mainframe, text="Stop", command=lambda: stop_all(teleop_node), width=14, height=1, bd=4, activebackground='#E83637')
btn_emergency = tk.Button(mainframe, text="Emergency Climb", command=lambda: emergencia(teleop_node), width=14, height=1, bd=4, activebackground='#E0E62E')

# Label para la imagen
image_label = ttk.Label(mainframe)

# Ubicaciones todas las secciones en el grid
mainframe.grid(column=0, row=0)

slider_vpitch.grid(column=0, row=0, columnspan=2, padx=10)
slider_vyaw.grid(column=5, row=0, columnspan=2, padx=10)

slider_vx.grid(column=0, row=1, columnspan=2, rowspan=4, pady=10)
slider_vz.grid(column=5, row=1, columnspan=2, rowspan=4, pady=10)

btn_start.grid(column=2, row=3, padx=5, pady=5)
btn_shutdown.grid(column=4, row=3, padx=5, pady=5)

btn_stop.grid(column=2, row=4, pady=10)
btn_emergency.grid(column=4, row=4, pady=10)

image_label.grid(column=2, row=0, columnspan=3, rowspan=3, pady=10)

# Desactivar botones diferentes al start al iniciar
toggle_controls('disabled')

# Actualizar la imagen en el label
root.after(0, update_image_label, image_label, image_node)

# Función para ejecutar el spin de ROS2 en un thread
def run_ros_spin():
    rclpy.spin(image_node)

ros_thread = threading.Thread(target=run_ros_spin, args=())
ros_thread.start()

root.mainloop()

teleop_node.destroy_node()
image_node.destroy_node()
rclpy.shutdown()
ros_thread.join()
