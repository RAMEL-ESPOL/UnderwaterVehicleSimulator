#include "rov_teleoperation/control_profundidad.hpp"

namespace rov_control_profundidad {

ControlProfundidad::ControlProfundidad(rclcpp::Node::SharedPtr node)
    : node_(node), target_profundidad_(0.0), current_profundidad_(0.0),
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0),
      kp_(150.0), ki_(1.0), kd_(140.0), calculated_thrust_(0.0) {
    initializePublishers();
}

void ControlProfundidad::setTargetProfundidad(double profundidad) {
    target_profundidad_ = profundidad;
}

void ControlProfundidad::updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg) {
    current_profundidad_ = odometry_msg->pose.pose.position.z;
    error_ = target_profundidad_ - current_profundidad_;
    integral_ += error_;
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;

    std_msgs::msg::Float64 target_profundidad_msg;
    target_profundidad_msg.data = target_profundidad_;
    pub_target_profundidad_->publish(target_profundidad_msg);

    computePID();
    publishThrustCommands();
}

void ControlProfundidad::initializePublishers() {
    propulsor_publishers_[0] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_left/cmd_thrust", 10);
    propulsor_publishers_[1] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_right/cmd_thrust", 10);
    propulsor_publishers_[2] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_center_thrust/cmd_thrust", 10);
    
    pub_target_profundidad_ = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/longitud_controller/target_altitud", 10);
}

void ControlProfundidad::computePID() {
    double thrust = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    // Normalizar o limitar el valor de 'thrust' si es necesario
    calculated_thrust_ = thrust;
}

void ControlProfundidad::limitValue(double& value, bool is_center) {
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

void ControlProfundidad::publishThrustCommands() {
    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = calculated_thrust_;

    for (auto& publisher : propulsor_publishers_) {
        if (publisher == propulsor_publishers_[2]) {
            thrust_msg.data = thrust_msg.data * 1.23;
            limitValue(thrust_msg.data, true);
        }else{
            limitValue(thrust_msg.data);
        }
        publisher->publish(thrust_msg);
    }
}

} // namespace rov_control_profundidad
