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
    void teleop()
    {
        int key;
        int left_count = 0;
        int right_count = 0;
        int vert_left_count = 0;
        int vert_right_count = 0;

        double left_thrust = 0.0;
        double right_thrust = 0.0;
        double vert_left_thrust = 0.0;
        double vert_right_thrust = 0.0;

        double escala = 0.1;

        while (true)
        {
            clear(); // Limpia la pantalla
            mvprintw(0, 0, "Control del Rov Max:");
            mvprintw(1, 0, "--------------------------------------------------");
            mvprintw(2, 0, "Presiona 'w' para girar propulsor trasero izquierdo");
            mvprintw(3, 0, "Presiona 's' para invertir giro propulsor trasero izquierdo");
            mvprintw(4, 0, "Presiona 'i' para girar propulsor trasero derecho");
            mvprintw(5, 0, "Presiona 'k' para invertir giro propulsor trasero derecho");
            mvprintw(6, 0, "Presiona 'a' para girar propulsor delantero izquierdo");
            mvprintw(7, 0, "Presiona 'd' para invertir giro propulsor delantero izquierdo");
            mvprintw(8, 0, "Presiona 'j' para girar propulsor delantero derecho");
            mvprintw(9, 0, "Presiona 'l' para invertir giro propulsor delantero derecho");
            mvprintw(10, 0, "Presiona 'q' para salir");
            mvprintw(11, 0, "--------------------------------------------------");
            mvprintw(12, 0, "Velocidad propulsor trasero izquierdo: %f", left_thrust);
            mvprintw(13, 0, "Velocidad propulsor trasero derecho: %f", right_thrust);
            mvprintw(14, 0, "Velocidad propulsor delantero izquierdo: %f", vert_left_thrust);
            mvprintw(15, 0, "Velocidad propulsor delantero derecho: %f", vert_right_thrust);
            mvprintw(16, 0, "Pulse una tecla para mover el Rov Max...   ");

            key = getch();

            switch (key)
            {
                // Teclas para subir y bajar la velocidad de cada propulsor
                case 'w':
                    left_count++;
                    left_thrust = escala * left_count;
                    publishThrust(pub_left_thrust_, left_thrust);
                    break;
                case 's':
                    left_count--;
                    left_thrust = escala * left_count;
                    publishThrust(pub_left_thrust_, left_thrust);
                    break;
                case 'i':
                    right_count++;
                    right_thrust = escala * right_count;
                    publishThrust(pub_right_thrust_, right_thrust);
                    break;
                case 'k':
                    right_count--;
                    right_thrust = escala * right_count;
                    publishThrust(pub_right_thrust_, right_thrust);
                    break;
                case 'a':
                    vert_left_count++;
                    vert_left_thrust = escala * vert_left_count;
                    publishThrust(pub_vert_left_thrust_, vert_left_thrust);
                    break;
                case 'd':
                    vert_left_count--;
                    vert_left_thrust = escala * vert_left_count;
                    publishThrust(pub_vert_left_thrust_, vert_left_thrust);
                    break;
                case 'j':
                    vert_right_count++;
                    vert_right_thrust = escala * vert_right_count;
                    publishThrust(pub_vert_right_thrust_, vert_right_thrust);
                    break;
                case 'l':
                    vert_right_count--;
                    vert_right_thrust = escala * vert_right_count;
                    publishThrust(pub_vert_right_thrust_, vert_right_thrust);
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto teleop_submarine = std::make_shared<TeleopSubmarine>();
    rclcpp::spin(teleop_submarine);
    rclcpp::shutdown();
    return 0;
}
