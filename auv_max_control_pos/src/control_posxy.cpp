#include "auv_max_control_pos/control_posxy.hpp"

namespace auv_control_posxy {

ControlPosXY::ControlPosXY(rclcpp::Node::SharedPtr node)
    : node_(node), target_posXY_(0.0), current_posXY_(0.0),
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0),
      kp_(40.0), ki_(0.05), kd_(39.0), max_integral_(50.0),
      derivative_filter_(0.45), calculated_thrust_(0.0) {
    initializePublishers();
}

void ControlPosXY::setTargetPosXY(double posXY) {
    target_posXY_ = posXY;
}

void ControlPosXY::updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg) {
    current_posXY_ = odometry_msg->pose.pose.position.x;
    error_ = target_posXY_ - current_posXY_;
    integral_ = std::max(std::min(integral_ + error_, max_integral_), -max_integral_);
    derivative_ = derivative_filter_ * derivative_ + (1 - derivative_filter_) * (error_ - prev_error_);
    prev_error_ = error_;

    std_msgs::msg::Float64 target_posXY_msg;
    target_posXY_msg.data = target_posXY_;
    pub_target_posXY_->publish(target_posXY_msg);

    computePID();
    publishThrustCommands();
}

void ControlPosXY::initializePublishers() {
    propulsor_publishers_[0] = node_->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_right_thrust/cmd_thrust", 10);
    propulsor_publishers_[1] = node_->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_left_thrust/cmd_thrust", 10);

    pub_target_posXY_ = node_->create_publisher<std_msgs::msg::Float64>("/model/auv_max/longitud_controller/target_longitud", 10);
}

void ControlPosXY::computePID() {
    double thrust = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    // Normalizar o limitar el valor de 'thrust' si es necesario
    calculated_thrust_ = thrust;
}

void ControlPosXY::limitValue(double& value) {
    if (value < MIN_THRUST_) {
        value = MIN_THRUST_;
    } else if (value > MAX_THRUST_) {
        value = MAX_THRUST_;
    }
}

void ControlPosXY::publishThrustCommands() {
    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = calculated_thrust_;

    limitValue(thrust_msg.data);

    if (std::abs(error_) < 0.3) {
        thrust_msg.data *= 0.25;
    } else if ((std::abs(error_) >= 0.3) && (std::abs(error_) < 0.5)) {
        thrust_msg.data *= 0.5;
    }

    for (auto& publisher : propulsor_publishers_) {
        publisher->publish(thrust_msg);
    }
}

} // namespace auv_control_posxy