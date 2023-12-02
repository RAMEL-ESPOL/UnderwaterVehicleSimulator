#ifndef ROV_ALTITUDE_CONTROL_ALTITUDE_CONTROLLER_HPP_
#define ROV_ALTITUDE_CONTROL_ALTITUDE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rov_altitude_control {

class AltitudeController {
public:
    AltitudeController(rclcpp::Node::SharedPtr node);
    void setTargetAltitude(double altitude);
    void updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg);

private:
    rclcpp::Node::SharedPtr node_;
    double target_altitude_;
    double current_altitude_;
    double error_, prev_error_, integral_, derivative_;
    double kp_, ki_, kd_; // PID parameters
    double calculated_thrust_;
    double MAX_THRUST_ = 2.5;
    double MIN_THRUST_ = -2.5;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_altitud_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 3> propulsor_publishers_;
    void initializePublishers();
    void computePID();
    void limitValue(double& value, bool is_center=false);
    void publishThrustCommands();
};

} // namespace rov_altitude_control

#endif // ROV_ALTITUDE_CONTROL_ALTITUDE_CONTROLLER_HPP_
