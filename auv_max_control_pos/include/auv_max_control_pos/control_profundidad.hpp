#ifndef AUV_PROFUNDIDAD_CONTROL_PROFUNDIDAD_CONTROLLER_HPP_
#define AUV_PROFUNDIDAD_CONTROL_PROFUNDIDAD_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

namespace auv_control_profundidad {

class ControlProfundidad {
public:
    ControlProfundidad(rclcpp::Node::SharedPtr node);
    void setTargetProfundidad(double profundidad);
    void updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg);

private:
    rclcpp::Node::SharedPtr node_;
    double target_profundidad_;
    double current_profundidad_;
    double error_, prev_error_, integral_, derivative_;
    double kp_, ki_, kd_; // PID parameters
    double calculated_thrust_;
    double MAX_THRUST_ = 2.5;
    double MIN_THRUST_ = -2.5;
    double max_integral_;
    double derivative_filter_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_profundidad_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 3> propulsor_publishers_;
    void initializePublishers();
    void computePID();
    void limitValue(double& value, bool is_center=false);
    void publishThrustCommands();
};

} // namespace auv_control_profundidad

#endif // AUV_PROFUNDIDAD_CONTROL_PROFUNDIDAD_CONTROLLER_HPP_
