#ifndef ROV_PITCH_CONTROL_PITCH_CONTROLLER_HPP_
#define ROV_PITCH_CONTROL_PITCH_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rov_control_pitch {

class ControlPitch {
public:
    ControlPitch(rclcpp::Node::SharedPtr node);
    void setTargetPitch(double pitch);
    void updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg);

private:
    rclcpp::Node::SharedPtr node_;
    double target_pitch_;
    double current_pitch_;
    double error_, prev_error_, integral_, derivative_;
    double kp_, ki_, kd_; // PID parameters
    double calculated_thrust_;
    double MAX_THRUST_ = 2.5;
    double MIN_THRUST_ = -2.5;
    double MAX_PITCH_ = 0.785398; // 45°
    double MIN_PITCH_ = -0.785398; // -45°
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_pitch_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 3> propulsor_publishers_;
    void initializePublishers();
    void computePID();
    void limitValue(double& value, bool is_center=false);
    void limitPitch(double& pitch);
    void publishThrustCommands();
};

} // namespace rov_control_pitch

#endif // ROV_PITCH_CONTROL_PITCH_CONTROLLER_HPP_
