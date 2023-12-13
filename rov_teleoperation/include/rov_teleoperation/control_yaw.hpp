#ifndef ROV_YAW_CONTROL_YAW_CONTROLLER_HPP_
#define ROV_YAW_CONTROL_YAW_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rov_control_yaw {

class ControlYaw {
public:
    ControlYaw(rclcpp::Node::SharedPtr node);
    void setTargetYaw(double yaw);
    void updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg);

private:
    rclcpp::Node::SharedPtr node_;
    double target_yaw_;
    double current_yaw_;
    double error_, prev_error_, integral_, derivative_;
    double kp_, ki_, kd_; // PID parameters
    double calculated_thrust_;
    double MAX_THRUST_ = 2.5;
    double MIN_THRUST_ = -2.5;
    double MAX_YAW_ = 1.22175; // 70°
    double MIN_YAW_ = -1.22175; // -70°
    double max_integral_;
    double derivative_filter_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_yaw_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 2> propulsor_publishers_;
    void initializePublishers();
    void computePID();
    void limitValue(double& value);
    void limitYaw(double& yaw);
    void publishThrustCommands();
};

} // namespace rov_control_yaw

#endif // ROV_YAW_CONTROL_YAW_CONTROLLER_HPP_
