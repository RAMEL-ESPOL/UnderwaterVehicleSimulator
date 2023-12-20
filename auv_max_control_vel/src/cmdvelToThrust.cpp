#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CmdvelToThrust : public rclcpp::Node {
public:
    CmdvelToThrust() : Node("cmd_to_thrust") {
        // Crear los publicadores
        pub_pro_vert_l_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_vert_thrust_left/cmd_thrust", 10);
        pub_pro_vert_r_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_vert_thrust_right/cmd_thrust", 10);
        pub_pro_vert_c_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_center_thrust/cmd_thrust", 10);
        pub_pro_l_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_left_thrust/cmd_thrust", 10);
        pub_pro_r_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_right_thrust/cmd_thrust", 10);
    }

    void init() {
        cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/model/auv_max/cmd_vel", 10,
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

    const double MAX_THRUST_ = 2.5; // Velocidad máxima del propulsor

    // Función para publicar valores de empuje
    void publishThrust(double thrust, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher) {
        std_msgs::msg::Float64 msg;
        msg.data = thrust;
        publisher->publish(msg);
    }

    // Calcula las velocidades para los propulsores verticales
    std::tuple<double, double, double> calculateVerticalThrusters(double linear_z, double angular_y) {
        double vel_pvc = std::clamp((linear_z + angular_y)*1.2, -MAX_THRUST_*1.2, MAX_THRUST_*1.2);
        double vel_pvi = std::clamp(linear_z - angular_y, -MAX_THRUST_, MAX_THRUST_);
        double vel_pvd = std::clamp(linear_z - angular_y, -MAX_THRUST_, MAX_THRUST_);
        return std::make_tuple(vel_pvc, vel_pvi, vel_pvd);
    }

    // Calcula las velocidades para los propulsores horizontales
    std::tuple<double, double> calculateHorizontalThrusters(double linear_x, double angular_z) {
        double vel_pi = std::clamp(linear_x + angular_z, -MAX_THRUST_, MAX_THRUST_);
        double vel_pd = std::clamp(linear_x - angular_z, -MAX_THRUST_, MAX_THRUST_);
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
