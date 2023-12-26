#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <tuple>

class CmdVelThrustConverter : public rclcpp::Node {
public:
    CmdVelThrustConverter();

    void init();

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

    void publishThrust(double thrust, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher);
    void timer_callback();
    void publishJointStates();
    std::tuple<double, double, double> calculateVerticalThrusters(double linear_z, double angular_y);
    std::tuple<double, double> calculateHorizontalThrusters(double linear_x, double angular_z);
};