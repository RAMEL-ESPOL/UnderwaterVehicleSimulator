#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ncurses.h>

class ROVTeleop : public rclcpp::Node {
public:
    ROVTeleop() : Node("rov_teleop") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/rov_max/cmd_vel", 10);
        initscr(); // Inicializa ncurses
        cbreak(); // Desactiva el buffer de línea, captura señales
        noecho(); // No imprime las teclas presionadas
        keypad(stdscr, TRUE); // Habilita la captura de teclas especiales
    }

    ~ROVTeleop() {
        endwin(); // Finaliza el modo ncurses
    }

    void printMsg()
    {
        clear(); // Limpia la pantalla
        mvprintw(0, 15, "Control del Rov Max");
        mvprintw(1, 0, "--------------------------------------------------");
        mvprintw(2, 0, "Los propulsores traseros izquierdo (PTI) y derecho (PTD) permiten mover hacia delante o atras");
        mvprintw(3, 0, "El propulsor trasero central (PTC) permite mover hacia arriba o abajo");
        mvprintw(4, 0, "Los propulsores delanteros izquierdo (PTI) y derecho (PTD) permiten mover hacia arriba o abajo");
        mvprintw(5, 0, "Presiona 'd' para girar PTI");
        mvprintw(6, 0, "Presiona 'a' para invertir giro PTI");
        mvprintw(7, 0, "Presiona 'l' para girar PTD");
        mvprintw(8, 0, "Presiona 'j' para invertir giro PTD");
        mvprintw(9, 0, "Presiona 'w' para girar PDI");
        mvprintw(10, 0, "Presiona 's' para invertir giro PDI");
        mvprintw(11, 0, "Presiona 'i' para girar PDD");
        mvprintw(12, 0, "Presiona 'k' para invertir giro PDD");
        mvprintw(13, 0, "Presiona 't' para girar PTC");
        mvprintw(14, 0, "Presiona 'g' para invertir giro PTC");
        mvprintw(15, 0, "Presiona 'q' para salir");
        mvprintw(16, 0, "--------------------------------------------------");
        mvprintw(17, 0, "  ____________________   ");
        mvprintw(18, 0, " /                    \\ ");
        mvprintw(19, 0, "/                      \\");
        mvprintw(20, 0, "|  _____        _____  |");
        mvprintw(21, 0, "| | PTI | *PTC | PTD | |");
        mvprintw(22, 0, "|  -----        -----  |");
        mvprintw(23, 0, "|                      |    Vel. lineal x: %f", cmd_vel_.linear.x);
        mvprintw(24, 0, "|                      |    Vel. lineal z: %f", cmd_vel_.linear.z);
        mvprintw(25, 0, "|                      |    Vel. angular y: %f", cmd_vel_.angular.y);
        mvprintw(26, 0, "|                      |    Vel. angular z: %f", cmd_vel_.angular.z);
        mvprintw(27, 0, "|                      |    -----------------------");
        mvprintw(28, 0, "|  _____        _____  |    Escala de velocidad: ");
        mvprintw(29, 0, "| | PDI | CAME | PDD | |");
        mvprintw(30, 0, "|  -----        -----  |");
        mvprintw(31, 0, "\\                     /");
        mvprintw(32, 0, " \\___________________/ ");
        mvprintw(33, 0, "--------------------------------------------------");
        mvprintw(34, 0, "Cambiar la escala de velocidad aumentar(m)/disminuir(n)");
        mvprintw(35, 0, "Para detener todos los propulsores pulse la letra 'c'");
        mvprintw(36, 0, "Pulse una tecla para mover el Rov Max...   ");
    }

    void run() {
        int ch;
        while (rclcpp::ok()) {
            printMsg();

            ch = getch(); // Lee una tecla

            switch (ch) {
                case KEY_UP:
                    increaseLinearZ();
                    break;
                case KEY_DOWN:
                    decreaseLinearZ();
                    break;
                case KEY_RIGHT:
                    increaseLinearX();
                    break;
                case KEY_LEFT:
                    decreaseLinearX();
                    break;
                case 'd':
                    increaseAngularZ();
                    break;
                case 'a':
                    decreaseAngularZ();
                    break;
                case 's':
                    increaseAngularY();
                    break;
                case 'w':
                    decreaseAngularY();
                    break;
                case 'q':
                    endwin();
                    return;
                default:
                    break;
            }
            publishCmdVel();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    geometry_msgs::msg::Twist cmd_vel_;
    const double LIMIT_VEL_LZ_ = 2.75;
    const double LIMIT_VEL_AY_ = 0.25;

    const double LIMIT_VEL_LX_ = 0.75;
    const double LIMIT_VEL_AZ_ = 1.75;

    void increaseLinearZ() {
        cmd_vel_.linear.z += 0.1;
        cmd_vel_.linear.z = std::clamp(cmd_vel_.linear.z, -LIMIT_VEL_LZ_, LIMIT_VEL_LZ_);
    }

    void decreaseLinearZ() {
        cmd_vel_.linear.z -= 0.1;
        cmd_vel_.linear.z = std::clamp(cmd_vel_.linear.z, -LIMIT_VEL_LZ_, LIMIT_VEL_LZ_);
    }

    void increaseLinearX() { 
        cmd_vel_.linear.x += 0.1; 
        cmd_vel_.linear.x = std::clamp(cmd_vel_.linear.x, -LIMIT_VEL_LX_, LIMIT_VEL_LX_);
    }
    void decreaseLinearX() { 
        cmd_vel_.linear.x -= 0.1;
        cmd_vel_.linear.x = std::clamp(cmd_vel_.linear.x, -LIMIT_VEL_LX_, LIMIT_VEL_LX_);
    }

    void increaseAngularZ() { 
        cmd_vel_.angular.z += 0.1; 
        cmd_vel_.angular.z = std::clamp(cmd_vel_.angular.z, -LIMIT_VEL_AZ_, LIMIT_VEL_AZ_);
    }
    void decreaseAngularZ() { 
        cmd_vel_.angular.z -= 0.1; 
        cmd_vel_.angular.z = std::clamp(cmd_vel_.angular.z, -LIMIT_VEL_AZ_, LIMIT_VEL_AZ_);
    }

    void increaseAngularY() {
        cmd_vel_.angular.y += 0.1;
        cmd_vel_.angular.y = std::clamp(cmd_vel_.angular.y, -LIMIT_VEL_AY_, LIMIT_VEL_AY_);
    }

    void decreaseAngularY() {
        cmd_vel_.angular.y -= 0.1;
        cmd_vel_.angular.y = std::clamp(cmd_vel_.angular.y, -LIMIT_VEL_AY_, LIMIT_VEL_AY_);
    }

    void publishCmdVel() {
        cmd_vel_pub_->publish(cmd_vel_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROVTeleop>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
