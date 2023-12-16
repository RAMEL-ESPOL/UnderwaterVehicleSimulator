#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CmdvelToThrust : public rclcpp::Node {
public:
    CmdvelToThrust() : Node("cmd_to_thrust") {
        // Crear los publicadores
        pub_pro_vert_l_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_left/cmd_thrust", 10);
        pub_pro_vert_r_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_right/cmd_thrust", 10);
        pub_pro_vert_c_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_center_thrust/cmd_thrust", 10);
        pub_pro_l_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_left_thrust/cmd_thrust", 10);
        pub_pro_r_ = this->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_right_thrust/cmd_thrust", 10);
    }

    void init() {
        cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/model/rov_max/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::cout << "Received cmd_vel message" << std::endl;
                std::cout << "Vel linear x: " << msg->linear.x << std::endl;
                std::cout << "Vel linear y: " << msg->linear.y << std::endl;
                std::cout << "Vel linear z: " << msg->linear.z << std::endl;
                std::cout << "Vel angular z: " << msg->angular.z << std::endl;
                std::cout << "Vel angular y: " << msg->angular.y << std::endl;
                std::cout << "Vel angular x: " << msg->angular.x << std::endl;

                // Calcular las velocidades para cada grupo de propulsores
                auto [vel_pvc, vel_pvi, vel_pvd] = calculateVerticalThrusters(msg->linear.z, msg->angular.y);
                auto [vel_pi, vel_pd] = calculateHorizontalThrusters(msg->linear.x, msg->angular.z);

                // Publicar las velocidades calculadas a cada propulsor
                publishThrust(vel_pvc, pub_pro_vert_c_);
                publishThrust(vel_pvi, pub_pro_vert_l_);
                publishThrust(vel_pvd, pub_pro_vert_r_);
                publishThrust(vel_pi, pub_pro_l_);
                publishThrust(vel_pd, pub_pro_r_);
            });
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_l_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_r_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_c_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_l_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_r_;

    // Función para publicar valores de empuje
    void publishThrust(double thrust, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher) {
        std_msgs::msg::Float64 msg;
        msg.data = thrust;
        publisher->publish(msg);
    }

    // Calcula las velocidades para los propulsores verticales
    std::tuple<double, double, double> calculateVerticalThrusters(double linear_z, double angular_y) {
        // [Lógica para calcular las velocidades de los propulsores verticales...]
        double base_thrust = linear_z / 3;  // Dividir igualmente la velocidad lineal en Z
        // Ajustar basado en la velocidad angular en Y
        // Esto es un ejemplo y puede necesitar ser ajustado
        double vel_pvc = base_thrust + angular_y / 2;
        double vel_pvi = base_thrust - angular_y / 4;
        double vel_pvd = base_thrust - angular_y / 4;
        return std::make_tuple(vel_pvc, vel_pvi, vel_pvd);
    }

    // Calcula las velocidades para los propulsores horizontales
    std::tuple<double, double> calculateHorizontalThrusters(double linear_x, double angular_z) {
        // [Lógica para calcular las velocidades de los propulsores horizontales...]
        double base_thrust = linear_x / 2;  // Dividir igualmente la velocidad lineal en X
        // Ajustar basado en la velocidad angular en Z
        // Esto es un ejemplo y puede necesitar ser ajustado
        double vel_pi = base_thrust + angular_z / 2;
        double vel_pd = base_thrust - angular_z / 2;
        return std::make_tuple(vel_pi, vel_pd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdvelToThrust>();
    node->init();  // Llama a la función init después de la creación del nodo
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
