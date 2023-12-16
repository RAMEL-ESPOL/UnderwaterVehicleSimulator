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

    void run() {
        int ch;
        while (rclcpp::ok()) {
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
                case 'a':
                    increaseAngularZ();
                    break;
                case 'd':
                    decreaseAngularZ();
                    break;
                case 'w':
                    increaseAngularY();
                    break;
                case 's':
                    decreaseAngularY();
                    break;
                default:
                    break;
            }
            publishCmdVel();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    geometry_msgs::msg::Twist cmd_vel_;

    void increaseLinearZ() { cmd_vel_.linear.z += 0.1; }
    void decreaseLinearZ() { cmd_vel_.linear.z -= 0.1; }
    void increaseLinearX() { cmd_vel_.linear.x += 0.1; }
    void decreaseLinearX() { cmd_vel_.linear.x -= 0.1; }
    void increaseAngularZ() { cmd_vel_.angular.z += 0.1; }
    void decreaseAngularZ() { cmd_vel_.angular.z -= 0.1; }
    void increaseAngularY() { cmd_vel_.angular.y += 0.1; }
    void decreaseAngularY() { cmd_vel_.angular.y -= 0.1; }

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
