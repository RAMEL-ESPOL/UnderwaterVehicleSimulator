#ifndef ROV_CONTROL_VEL_CMDVEL_TO_THRUST_HPP_
#define ROV_CONTROL_VEL_CMDVEL_TO_THRUST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace rov_control_vel {
class CmdvelToThrust {
public:
    CmdvelToThrust(rclcpp::Node::SharedPtr node);
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_l_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_r_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_vert_c_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_l_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pro_r_;
};
}

#endif // ROV_CONTROL_VEL_CMDVEL_TO_THRUST_HPP_