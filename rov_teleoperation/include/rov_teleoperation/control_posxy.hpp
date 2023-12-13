#ifndef ROV_POSXY_CONTROL_POSXY_CONTROLLER_HPP_
#define ROV_POSXY_CONTROL_POSXY_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rov_control_posxy {

class ControlPosXY {
public:
    ControlPosXY(rclcpp::Node::SharedPtr node);
    void setTargetPosXY(double posXY);
    void updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg);

private:
    rclcpp::Node::SharedPtr node_;
    double target_posXY_;
    double current_posXY_;
    double error_, prev_error_, integral_, derivative_;
    double kp_, ki_, kd_; // PID parameters
    double calculated_thrust_;
    double MAX_THRUST_ = 2.5;
    double MIN_THRUST_ = -2.5;
    double max_integral_;
    double derivative_filter_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_posXY_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 2> propulsor_publishers_;
    void initializePublishers();
    void computePID();
    void limitValue(double& value);
    void publishThrustCommands();
};

} // namespace rov_control_posxy

#endif // ROV_POSXY_CONTROL_POSXY_CONTROLLER_HPP_
