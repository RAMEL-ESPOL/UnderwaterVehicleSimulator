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
        self.pub_right = self.create_publisher(Float64, '/model/rov_max/joint/shell_to_right_thrust/cmd_thrust', 10)
        self.pub_left = self.create_publisher(Float64, '/model/rov_max/joint/shell_to_left_thrust/cmd_thrust', 10)

        self.escala = 0.1
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.vert_center_thrust = 0.0
        self.vert_left_thrust = 0.0
        self.vert_right_thrust = 0.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def printMsg(self):
        print(f'--------------------------------------------------') 
        print(f'Control del Rov Max') 
        print(f'Presiona w para subir') 
        print(f'Presiona s para bajar')
        print(f'Presiona a para avanzar')
        print(f'Presiona d para retroceder')
        print(f'Presiona p para detener propulsores')
        print(f'--------------------------------------------------') 
        print(f'  ____________________  ') 
        print(f' /                    \\') 
        print(f'/                      \\') 
        print(f'|  _____        _____  |') 
        print(f'| | PTI | *PTC | PTD | |') 
        print(f'|  -----        -----  |') 
        print(f'|                      |') 
        print(f'|                      |') 
        print(f'|                      |') 
        print(f'|                      |') 
        print(f'|                      |') 
        print(f'|  _____        _____  |') 
        print(f'| | PDI | CAME | PDD | |') 
        print(f'|  -----        -----  |') 
        print(f'\\                     /') 
        print(f' \\___________________/') 
        print(f'Cambiar la escala de velocidad aumentar(m)/disminuir(n)') 
        print(f'--------------------------------------------------') 

    def printValoresMtrs(self):
        print(f'Vel. PTI: {self.left_thrust}') 
        print(f'Vel. PTD: {self.right_thrust}') 
        print(f'Vel. PTC: {self.vert_center_thrust}') 
        print(f'Vel. PDI: {self.vert_left_thrust}') 
        print(f'Vel. PDD: {self.vert_right_thrust}') 
        print(f'Escala de velocidad: {self.escala}') 

    def subir(self):
        self.vert_left_thrust += self.escala
        self.vert_right_thrust += self.escala
        self.vert_center_thrust += self.escala*1.25

        self.pub_vert_left.publish(Float64(data=self.vert_left_thrust))
        self.pub_vert_right.publish(Float64(data=self.vert_right_thrust))
        self.pub_vert_centrl.publish(Float64(data=self.vert_center_thrust))

        self.printValoresMtrs()
    
    def bajar(self):
        self.vert_left_thrust -= self.escala
        self.vert_right_thrust -= self.escala
        self.vert_center_thrust -= self.escala*1.25

        self.pub_vert_left.publish(Float64(data=self.vert_left_thrust))
        self.pub_vert_right.publish(Float64(data=self.vert_right_thrust))
        self.pub_vert_centrl.publish(Float64(data=self.vert_center_thrust))

        self.printValoresMtrs()

    def avanzar(self):
        self.left_thrust += self.escala
        self.right_thrust += self.escala

        self.pub_left.publish(Float64(data=self.left_thrust))
        self.pub_right.publish(Float64(data=self.right_thrust))

        self.printValoresMtrs()

    def retroceder(self):
        self.left_thrust -= self.escala
        self.right_thrust -= self.escala

        self.pub_left.publish(Float64(data=self.left_thrust))
        self.pub_right.publish(Float64(data=self.right_thrust))

        self.printValoresMtrs()

    def detener(self):
        self.vert_left_thrust = 0.0
        self.vert_right_thrust = 0.0
        self.vert_center_thrust = 0.0
        self.left_thrust = 0.0
        self.right_thrust = 0.0

        self.pub_vert_left.publish(Float64(data=self.vert_left_thrust))
        self.pub_vert_right.publish(Float64(data=self.vert_right_thrust))
        self.pub_vert_centrl.publish(Float64(data=self.vert_center_thrust))
        self.pub_left.publish(Float64(data=self.left_thrust))
        self.pub_right.publish(Float64(data=self.right_thrust))

        self.printValoresMtrs()

    def aumentarEscala(self):
        self.escala += 0.1

        self.printValoresMtrs()
    
    def disminuirEscala(self):
        self.escala -= 0.1

        self.printValoresMtrs()

    def run(self):
        estado_msg = 0
        self.printMsg()

        while rclpy.ok():

            key = self.get_key()

            if key == 'w':
                self.subir()

                estado_msg += 1
            elif key == 's':
                self.bajar()

                estado_msg += 1
            elif key == 'a':
                self.avanzar()

                estado_msg += 1
            elif key == 'd':
                self.retroceder()

                estado_msg += 1
            elif key == 'm':
                self.aumentarEscala()

                estado_msg += 1
            elif key == 'n':
                self.disminuirEscala()
                
                estado_msg += 1
            elif key == 'p':
                self.detener()

                estado_msg += 1
            elif key == 'c':
                break

            if estado_msg == 4:
                estado_msg = 0
                self.printMsg()
                
def main(args=None):
    rclpy.init(args=args)
    rov_teleop_node = ROVTeleopNode()
    rov_teleop_node.run()

if __name__ == '__main__':
    main()
