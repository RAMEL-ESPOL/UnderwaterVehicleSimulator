import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty

class ROVTeleopNode(Node):
    def __init__(self):
        super().__init__('rov_teleop_node')
        # Definir publicadores para cada conjunto de propulsores
        self.pub_vert_left = self.create_publisher(Float64, '/model/rov_max/joint/shell_to_vert_thrust_left/cmd_thrust', 10)
        self.pub_vert_right = self.create_publisher(Float64, '/model/rov_max/joint/shell_to_vert_thrust_right/cmd_thrust', 10)
        self.pub_vert_centrl = self.create_publisher(Float64, '/model/rov_max/joint/shell_to_center_thrust/cmd_thrust', 10)

        # Mapeo de teclas a movimientos
        self.key_mapping = {
            'w': (1.0, 0),  # Ascender
            's': (-1.0, 0), # Descender
        }

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key in self.key_mapping:
                # Lógica para controlar los propulsores basada en la tecla presionada
                vertical, horizontal = self.key_mapping[key]
                self.pub_vert_left.publish(Float64(data=vertical))
                self.pub_vert_right.publish(Float64(data=vertical))
                self.pub_vert_centrl.publish(Float64(data=vertical*1.25))

def main(args=None):
    rclpy.init(args=args)
    rov_teleop_node = ROVTeleopNode()
    rov_teleop_node.run()

if __name__ == '__main__':
    main()
