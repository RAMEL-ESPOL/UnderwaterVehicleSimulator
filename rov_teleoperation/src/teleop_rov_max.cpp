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

        while (true)
        {
            key = getch();

            switch (key)
            {
                // Teclas para subir y bajar la velocidad de cada propulsor
                case 'w':
                    left_count++;
                    publishThrust(pub_left_thrust_, "left", 0.1 * left_count);
                    break;
                case 's':
                    left_count--;
                    publishThrust(pub_left_thrust_, "left", 0.1 * left_count);
                    break;
                case 'i':
                    right_count++;
                    publishThrust(pub_right_thrust_, "right", 0.1 * right_count);
                    break;
                case 'k':
                    right_count--;
                    publishThrust(pub_right_thrust_, "right", 0.1 * right_count);
                    break;
                case 'a':
                    vert_left_count++;
                    publishThrust(pub_vert_left_thrust_, "vertical left", 0.1 * vert_left_count);
                    break;
                case 'd':
                    vert_left_count--;
                    publishThrust(pub_vert_left_thrust_, "vertical left", 0.1 * vert_left_count);
                    break;
                case 'j':
                    vert_right_count++;
                    publishThrust(pub_vert_right_thrust_, "vertical right", 0.1 * vert_right_count);
                    break;
                case 'l':
                    vert_right_count--;
                    publishThrust(pub_vert_right_thrust_, "vertical right", 0.1 * vert_right_count);
                    break;
                // Otras teclas o casos adicionales si es necesario
            }
        }
    }

    void publishThrust(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub, const std::string &propulsor, double velocidad)
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = velocidad;
        pub->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Propulsor %s: Velocidad %f", propulsor.c_str(), velocidad);
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
