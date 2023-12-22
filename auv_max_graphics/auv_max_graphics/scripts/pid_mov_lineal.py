import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

class PositionPlotter(Node):
    def __init__(self):
        super().__init__('position_plotter_lineal')
        self.target_posxy = self.create_subscription(
            Float64,
            '/model/auv_max/longitud_controller/target_longitud',
            self.target_callback_posxy,
            10
        )
        self.target_prof = self.create_subscription(
            Float64,
            '/model/auv_max/longitud_controller/target_altitud',
            self.target_callback_prof,
            10
        )
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/model/auv_max/odometry',
            self.odometry_callback,
            10
        )

        self.posxy_positions = []
        self.prof_positions = []
        self.target_position_posxy = None
        self.target_position_prof = None

        # Crear subplots
        self.fig, self.axes = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.tight_layout()

    def target_callback_posxy(self, msg):
        self.target_position_posxy = msg.data
    
    def target_callback_prof(self, msg):
        self.target_position_prof = msg.data

    def odometry_callback(self, msg):
        self.posxy_positions.append(msg.pose.pose.position.x)
        self.prof_positions.append(msg.pose.pose.position.z)
        self.plot_positions()

    def plot_positions(self):
        self.axes[0].cla()
        self.axes[0].plot(self.posxy_positions, label='Actual Position PosXY')
        self.axes[0].set_ylabel('PosXY')
        self.axes[0].set_xlabel('Time')

        self.axes[1].cla()
        self.axes[1].plot(self.prof_positions, label='Actual Position Profundidad')
        self.axes[1].set_ylabel('Profundidad')
        self.axes[1].set_xlabel('Time')

        if self.target_position_posxy is not None:
            self.axes[0].axhline(y=self.target_position_posxy, color='b', linestyle='--', label='Target Position PosXY')
        if self.target_position_prof is not None:
            self.axes[1].axhline(y=self.target_position_prof, color='y', linestyle='--', label='Target Position Profundidad')

        for ax in self.axes:
            ax.legend()
            ax.grid(True)

        self.fig.canvas.flush_events()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    position_plotter = PositionPlotter()
    rclpy.spin(position_plotter)
    position_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()