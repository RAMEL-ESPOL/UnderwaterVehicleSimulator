#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>

class CmdvelToThrust : public rclcpp::Node {
public:
    CmdvelToThrust() : Node("cmd_to_thrust"), last_update_time_(this->get_clock()->now()) {
        // Crear los publicadores
        pub_pro_vert_l_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_vert_thrust_left/cmd_thrust", 10);
        pub_pro_vert_r_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_vert_thrust_right/cmd_thrust", 10);
        pub_pro_vert_c_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_center_thrust/cmd_thrust", 10);
        pub_pro_l_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_left_thrust/cmd_thrust", 10);
        pub_pro_r_ = this->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_right_thrust/cmd_thrust", 10);

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        joint_names_ = {"shell_to_vert_thrust_left", "shell_to_vert_thrust_right", 
                        "shell_to_center_thrust", "shell_to_left_thrust", 
                        "shell_to_right_thrust"};

        // Inicializar el estado de las articulaciones
        joint_positions_ = std::vector<double>(joint_names_.size(), 0.0);
        joint_velocities_ = std::vector<double>(joint_names_.size(), 0.0);

    }

    void init() {
        cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/model/auv_max/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::cout << "Received cmd_vel message" << std::endl;

                // Calcular las velocidades para cada grupo de propulsores
                auto [vel_pvc, vel_pvi, vel_pvd] = calculateVerticalThrusters(msg->linear.z, msg->angular.y);
                auto [vel_pi, vel_pd] = calculateHorizontalThrusters(msg->linear.x, msg->angular.z);

                // Publicar las velocidades calculadas a cada propulsor
                publishThrust(vel_pvc, pub_pro_vert_c_);
                publishThrust(vel_pvi, pub_pro_vert_l_);
                publishThrust(vel_pvd, pub_pro_vert_r_);
                publishThrust(vel_pi, pub_pro_l_);
                publishThrust(vel_pd, pub_pro_r_);

                joint_velocities_[0] += vel_pvi; 
                joint_velocities_[1] += vel_pvd; 
                joint_velocities_[2] += vel_pvc; 
                joint_velocities_[3] += vel_pi; 
                joint_velocities_[4] += vel_pd; 

                // Publicar las posiciones de las articulaciones
                publishJointStates();
            });
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Intervalo de 50 ms
            std::bind(&CmdvelToThrust::timer_callback, this));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_l_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_r_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_c_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_l_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_r_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;

    const double MAX_THRUST_ = 2.5; // Velocidad máxima del propulsor

    rclcpp::Time last_update_time_;

    rclcpp::TimerBase::SharedPtr timer_;


    // Función para publicar valores de empuje
    void publishThrust(double thrust, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher) {
        std_msgs::msg::Float64 msg;
        msg.data = thrust;
        publisher->publish(msg);
    }

    void timer_callback() {
        // Aquí se actualizará y publicará el estado de las articulaciones, incluso si cmd_vel no cambia.
        publishJointStates();
    }

    // Función para publicar el estado de las articulaciones
    void publishJointStates() {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // Integración simple para estimar la nueva posición del frame de los propulsores
        for (size_t i = 0; i < joint_positions_.size(); ++i) {
            joint_positions_[i] += joint_velocities_[i] * dt;
        }

        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = current_time;
        joint_state_msg.header.frame_id = "auv_max_frame";
        
        joint_state_msg.name = joint_names_;

        joint_state_msg.position = joint_positions_;
        joint_state_msg.velocity = joint_velocities_; 
        joint_state_msg.effort = std::vector<double>(joint_names_.size(), 0.0);

        joint_state_pub_->publish(joint_state_msg);
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
