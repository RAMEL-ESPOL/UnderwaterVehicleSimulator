#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ncurses.h>

class TeleopSubmarine : public rclcpp::Node
{
public:
    TeleopSubmarine() : Node("teleop_submarine")
    {
        // Crear los publicadores para los propulsores
        pub_left_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_left_thrust/cmd_thrust", 10);
        pub_right_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_right_thrust/cmd_thrust", 10);
        pub_vert_left_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_left/cmd_thrust", 10);
        pub_vert_right_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_right/cmd_thrust", 10);
        pub_vert_center_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_center_thrust/cmd_thrust", 10);

        // Iniciar la interfaz de ncurses
        initscr();
        raw();
        keypad(stdscr, TRUE);
        noecho();

        // Ejecutar la teleoperación
        teleop();
    }

    ~TeleopSubmarine()
    {
        // Detener la interfaz de ncurses
        endwin();
    }

private:
    void printMsg(double left_thrust, double right_thrust, double vert_left_thrust, double vert_right_thrust, double vert_center_thrust, double escala)
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
        mvprintw(23, 0, "|                      |    Vel. PTI: %f", left_thrust);
        mvprintw(24, 0, "|                      |    Vel. PTD: %f", right_thrust);
        mvprintw(25, 0, "|                      |    Vel. PTC: %f", vert_center_thrust);
        mvprintw(26, 0, "|                      |    Vel. PDI: %f", vert_left_thrust);
        mvprintw(27, 0, "|                      |    Vel. PDD: %f", vert_right_thrust);
        mvprintw(28, 0, "|  _____        _____  |    Escala de velocidad: %f", escala);
        mvprintw(29, 0, "| | PDI | CAME | PDD | |");
        mvprintw(30, 0, "|  -----        -----  |");
        mvprintw(31, 0, "\\                     /");
        mvprintw(32, 0, " \\___________________/ ");
        mvprintw(33, 0, "--------------------------------------------------");
        mvprintw(34, 0, "Cambiar la escala de velocidad aumentar(m)/disminuir(n)");
        mvprintw(35, 0, "Para detener todos los propulsores pulse la letra 'c'");
        mvprintw(36, 0, "Pulse una tecla para mover el Rov Max...   ");
    }

    void teleop()
    {
        int key;
        int left_count = 0;
        int right_count = 0;
        int vert_left_count = 0;
        int vert_right_count = 0;
        int vert_center_count = 0;

        double left_thrust = 0.0;
        double right_thrust = 0.0;
        double vert_left_thrust = 0.0;
        double vert_right_thrust = 0.0;
        double vert_center_thrust = 0.0;

        double escala = 0.1;
        const double cambiar_escala = 0.05;

        while (true)
        {
            printMsg(left_thrust, right_thrust, vert_left_thrust, vert_right_thrust, vert_center_thrust, escala);

            key = getch();

            switch (key)
            {
                // Teclas para subir y bajar la velocidad de cada propulsor
                case 'd':
                    left_count++;
                    left_thrust = escala * left_count;
                    publishThrust(pub_left_thrust_, left_thrust);
                    break;
                case 'a':
                    left_count--;
                    left_thrust = escala * left_count;
                    publishThrust(pub_left_thrust_, left_thrust);
                    break;
                case 'l':
                    right_count++;
                    right_thrust = escala * right_count;
                    publishThrust(pub_right_thrust_, right_thrust);
                    break;
                case 'j':
                    right_count--;
                    right_thrust = escala * right_count;
                    publishThrust(pub_right_thrust_, right_thrust);
                    break;
                case 's':
                    vert_left_count--;
                    vert_left_thrust = escala * vert_left_count;
                    publishThrust(pub_vert_left_thrust_, vert_left_thrust);
                    break;
                case 'w':
                    vert_left_count++;
                    vert_left_thrust = escala * vert_left_count;
                    publishThrust(pub_vert_left_thrust_, vert_left_thrust);
                    break;
                case 'k':
                    vert_right_count--;
                    vert_right_thrust = escala * vert_right_count;
                    publishThrust(pub_vert_right_thrust_, vert_right_thrust);
                    break;
                case 'i':
                    vert_right_count++;
                    vert_right_thrust = escala * vert_right_count;
                    publishThrust(pub_vert_right_thrust_, vert_right_thrust);
                    break;
                case 'g':
                    vert_center_count--;
                    vert_center_thrust = escala * vert_center_count;
                    publishThrust(pub_vert_center_thrust_, vert_center_thrust);
                    break;
                case 't':
                    vert_center_count++;
                    vert_center_thrust = escala * vert_center_count;
                    publishThrust(pub_vert_center_thrust_, vert_center_thrust);
                    break;
                case 'm':
                    escala = escala + cambiar_escala;
                    break;
                case 'n':
                    escala = escala - cambiar_escala;
                    break;
                case 'c':
                    left_thrust = 0.0;
                    right_thrust = 0.0;
                    vert_left_thrust = 0.0;
                    vert_right_thrust = 0.0;
                    vert_center_thrust = 0.0;
                    publishThrust(pub_left_thrust_, left_thrust);
                    publishThrust(pub_right_thrust_, right_thrust);
                    publishThrust(pub_vert_left_thrust_, vert_left_thrust);
                    publishThrust(pub_vert_right_thrust_, vert_right_thrust);
                    publishThrust(pub_vert_center_thrust_, vert_center_thrust);
                    break;
                case 'q':
                    endwin();
                    return;
                // Otras teclas o casos adicionales si es necesario
            }
        }
    }

    void publishThrust(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub, double velocidad)
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = velocidad;
        pub->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_right_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_center_thrust_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto teleop_submarine = std::make_shared<TeleopSubmarine>();
    rclcpp::spin(teleop_submarine);
    rclcpp::shutdown();
    return 0;
}
