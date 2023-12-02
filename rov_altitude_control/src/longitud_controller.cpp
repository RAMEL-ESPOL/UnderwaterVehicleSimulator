#include "rov_altitude_control/longitud_controller.hpp"

namespace rov_longitud_control {

LongitudController::LongitudController(rclcpp::Node::SharedPtr node)
    : node_(node), target_longitud_(0.0), current_longitud_(0.0),
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0),
      kp_(550.0), ki_(0.0), kd_(540.0), calculated_thrust_(0.0) {
    initializePublishers();
}

void LongitudController::setTargetLongitud(double longitud) {
    target_longitud_ = longitud;
}

void LongitudController::updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg) {
    current_longitud_ = odometry_msg->pose.pose.position.x;
    error_ = target_longitud_ - current_longitud_;
    integral_ += error_;
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;

    std_msgs::msg::Float64 target_longitud_msg;
    target_longitud_msg.data = target_longitud_;
    pub_target_longitud_->publish(target_longitud_msg);

    computePID();
    publishThrustCommands();
}

void LongitudController::initializePublishers() {
    propulsor_publishers_[0] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_right_thrust/cmd_thrust", 10);
    propulsor_publishers_[1] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_left_thrust/cmd_thrust", 10);

    pub_target_longitud_ = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/longitud_controller/target_longitud", 10);
}

void LongitudController::computePID() {
    double thrust = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    // Normalizar o limitar el valor de 'thrust' si es necesario
    calculated_thrust_ = thrust;
}

void LongitudController::limitValue(double& value) {
    if (value < MIN_THRUST_) {
        value = MIN_THRUST_;
    } else if (value > MAX_THRUST_) {
        value = MAX_THRUST_;
    }
}

void LongitudController::publishThrustCommands() {
    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = calculated_thrust_;

    for (auto& publisher : propulsor_publishers_) {
        /*if (publisher == propulsor_publishers_[2]) {
            thrust_msg.data = thrust_msg.data * 1.23;
        }*/
        RCLCPP_INFO(node_->get_logger(), "Thrust: %f", thrust_msg.data);
        limitValue(thrust_msg.data);
        publisher->publish(thrust_msg);
    }
}

} // namespace rov_altitude_control