#include "rov_teleoperation/control_posxy.hpp"

namespace rov_control_posxy {

ControlPosXY::ControlPosXY(rclcpp::Node::SharedPtr node)
    : node_(node), target_posXY_(0.0), current_posXY_(0.0),
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0),
      kp_(550.0), ki_(0.0), kd_(540.0), calculated_thrust_(0.0) {
    initializePublishers();
}

void ControlPosXY::setTargetPosXY(double posXY) {
    target_posXY_ = posXY;
}

void ControlPosXY::updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg) {
    current_posXY_ = odometry_msg->pose.pose.position.x;
    error_ = target_posXY_ - current_posXY_;
    integral_ += error_;
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;

    std_msgs::msg::Float64 target_posXY_msg;
    target_posXY_msg.data = target_posXY_;
    pub_target_posXY_->publish(target_posXY_msg);

    computePID();
    publishThrustCommands();
}

void ControlPosXY::initializePublishers() {
    propulsor_publishers_[0] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_right_thrust/cmd_thrust", 10);
    propulsor_publishers_[1] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_left_thrust/cmd_thrust", 10);

    pub_target_posXY_ = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/longitud_controller/target_longitud", 10);
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

    for (auto& publisher : propulsor_publishers_) {
        limitValue(thrust_msg.data);
        publisher->publish(thrust_msg);
    }
}

} // namespace rov_control_posxy