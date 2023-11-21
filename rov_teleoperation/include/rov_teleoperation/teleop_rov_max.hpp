#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class TeleopSubmarine : public rclcpp::Node
{
public:
    TeleopSubmarine();

    ~TeleopSubmarine();

private:
    void printMsg(double left_thrust, double right_thrust, double vert_left_thrust, double vert_right_thrust, double vert_center_thrust, double escala);

    void teleop();

    void increaseThrust(double* thrust, double escala, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub);

    void decreaseThrust(double* thrust, double escala, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub);

    void stopAllThrust(double* left_thrust, double* right_thrust, double* vert_left_thrust, double* vert_right_thrust, double* vert_center_thrust,
                       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right,
                       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_left, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_right,
                       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_center);

    void publishThrust(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub, double velocidad);

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_right_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vert_center_thrust_;
};
