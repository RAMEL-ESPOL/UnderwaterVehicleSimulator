import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

class PositionPlotter(Node):
    def __init__(self):
        super().__init__('position_plotter_angular')
        self.target_pitch = self.create_subscription(
            Float64,
            '/model/rov_max/longitud_controller/target_pitch',
            self.target_callback_pitch,
            10
        )
        self.target_yaw = self.create_subscription(
            Float64,
            '/model/rov_max/longitud_controller/target_yaw',
            self.target_callback_yaw,
            10
        )
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/model/rov_max/odometry',
            self.odometry_callback,
            10
        )
        self.pitch_positions = []
        self.yaw_positions = []
        self.target_position_pitch = None
        self.target_position_yaw = None

    def target_callback_pitch(self, msg):
        print(msg.data)
        self.target_position_pitch = msg.data
    
    def target_callback_yaw(self, msg):
        print(msg.data)
        self.target_position_yaw = msg.data

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def odometry_callback(self, msg):
        euler = self.euler_from_quaternion(msg.pose.pose.orientation)
        pitch = euler[1]
        yaw = euler[2]
        self.pitch_positions.append(pitch)
        self.yaw_positions.append(yaw)
        self.plot_positions()

    def plot_positions(self):
        plt.plot(self.pitch_positions, label='Actual Position Pitch')
        plt.plot(self.yaw_positions, label='Actual Position Yaw')
        if self.target_position_pitch is not None:
            plt.axhline(y=self.target_position_pitch, color='r', linestyle='--', label='Target Position Pitch')
        if self.target_position_yaw is not None:
            plt.axhline(y=self.target_position_yaw, color='g', linestyle='--', label='Target Position Yaw')
        plt.xlabel('Time')
        plt.ylabel('Position Angular')
        plt.legend()
        plt.grid(True)
        plt.pause(0.01)
        plt.clf()

def main(args=None):
    rclpy.init(args=args)
    position_plotter = PositionPlotter()
    rclpy.spin(position_plotter)
    position_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
