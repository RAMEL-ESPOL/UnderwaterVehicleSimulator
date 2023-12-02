#include "rov_altitude_control/altitude_controller.hpp"

namespace rov_altitude_control {

AltitudeController::AltitudeController(rclcpp::Node::SharedPtr node)
    : node_(node), target_altitude_(0.0), current_altitude_(0.0),
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0),
      kp_(150.0), ki_(1.0), kd_(140.0), calculated_thrust_(0.0) {
    initializePublishers();
}

void AltitudeController::setTargetAltitude(double altitude) {
    target_altitude_ = altitude;
}

void AltitudeController::updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg) {
    current_altitude_ = odometry_msg->pose.pose.position.z;
    error_ = target_altitude_ - current_altitude_;
    integral_ += error_;
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;

    std_msgs::msg::Float64 target_altitud_msg;
    target_altitud_msg.data = target_altitude_;
    pub_target_altitud_->publish(target_altitud_msg);

    computePID();
    publishThrustCommands();
}

void AltitudeController::initializePublishers() {
    propulsor_publishers_[0] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_left/cmd_thrust", 10);
    propulsor_publishers_[1] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_right/cmd_thrust", 10);
    propulsor_publishers_[2] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_center_thrust/cmd_thrust", 10);
    /*propulsor_publishers_[3] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_right_thrust/cmd_thrust", 10);
    propulsor_publishers_[4] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_left_thrust/cmd_thrust", 10);*/

    pub_target_altitud_ = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/longitud_controller/target_altitud", 10);
}

void AltitudeController::computePID() {
    double thrust = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    // Normalizar o limitar el valor de 'thrust' si es necesario
    calculated_thrust_ = thrust;
}

void AltitudeController::limitValue(double& value, bool is_center) {
    if (is_center) {
        if (value < MIN_THRUST_*1.25) {
            value = MIN_THRUST_*1.25;
        } else if (value > MAX_THRUST_*1.25) {
            value = MAX_THRUST_*1.25;
        }
    } else {
        if (value < MIN_THRUST_) {
            value = MIN_THRUST_;
        } else if (value > MAX_THRUST_) {
            value = MAX_THRUST_;
        }
    }
}

void AltitudeController::publishThrustCommands() {
    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = calculated_thrust_;

    for (auto& publisher : propulsor_publishers_) {
        if (publisher == propulsor_publishers_[2]) {
            thrust_msg.data = thrust_msg.data * 1.23;
            limitValue(thrust_msg.data, true);
        }else{
            limitValue(thrust_msg.data);
        }
        RCLCPP_INFO(node_->get_logger(), "Thrust: %f", thrust_msg.data);
        publisher->publish(thrust_msg);
    }
}

} // namespace rov_altitude_control
