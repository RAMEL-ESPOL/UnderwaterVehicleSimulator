#ifndef ROV_LONGITUD_CONTROL_LONGITUD_CONTROLLER_HPP_
#define ROV_LONGITUD_CONTROL_LONGITUD_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rov_longitud_control {

class LongitudController {
public:
    LongitudController(rclcpp::Node::SharedPtr node);
    void setTargetLongitud(double longitud);
    void updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg);

private:
    rclcpp::Node::SharedPtr node_;
    double target_longitud_;
    double current_longitud_;
    double error_, prev_error_, integral_, derivative_;
    double kp_, ki_, kd_; // PID parameters
    double calculated_thrust_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 2> propulsor_publishers_;
    void initializePublishers();
    void computePID();
    void publishThrustCommands();
};

} // namespace rov_altitude_control

#endif // ROV_ALTITUDE_CONTROL_ALTITUDE_CONTROLLER_HPP_
