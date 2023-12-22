#include "auv_max_control_pos/control_yaw.hpp"
#include <iostream>
#include <fstream>

namespace auv_control_yaw {

ControlYaw::ControlYaw(rclcpp::Node::SharedPtr node)
    : node_(node), target_yaw_(0.0), current_yaw_(0.0),
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0),
      kp_(20.0), ki_(0.0), kd_(18.0), max_integral_(25.0),
      derivative_filter_(0.3), calculated_thrust_(0.0) {
    initializePublishers();
}

void ControlYaw::setTargetYaw(double yaw) {
    target_yaw_ = yaw;
    limitYaw(target_yaw_);
}

void ControlYaw::updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(odometry_msg->pose.pose.orientation, quaternion);

    // Convertir a ángulos de Euler
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quaternion);
    m.getRPY(roll, pitch, yaw);

    yaw = std::round(yaw * 100.0) / 100.0;

    current_yaw_ = yaw;   

    error_ = target_yaw_ - current_yaw_;
    integral_ = std::max(std::min(integral_ + error_, max_integral_), -max_integral_);
    derivative_ = derivative_filter_ * derivative_ + (1 - derivative_filter_) * (error_ - prev_error_);
    prev_error_ = error_;

    std_msgs::msg::Float64 target_yaw_msg;
    target_yaw_msg.data = target_yaw_;
    pub_target_yaw_->publish(target_yaw_msg);

    computePID();
    publishThrustCommands();
}

void ControlYaw::initializePublishers() {
    propulsor_publishers_[0] = node_->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_right_thrust/cmd_thrust", 10);
    propulsor_publishers_[1] = node_->create_publisher<std_msgs::msg::Float64>("/model/auv_max/joint/shell_to_left_thrust/cmd_thrust", 10);

    pub_target_yaw_ = node_->create_publisher<std_msgs::msg::Float64>("/model/auv_max/longitud_controller/target_yaw", 10);
}

void ControlYaw::computePID() {
    double thrust = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    // Normalizar o limitar el valor de 'thrust' si es necesario
    calculated_thrust_ = thrust;
}

void ControlYaw::limitValue(double& value) {
    if (value < MIN_THRUST_) {
        value = MIN_THRUST_;
    } else if (value > MAX_THRUST_) {
        value = MAX_THRUST_;
    }
}

void ControlYaw::limitYaw(double& yaw) {
    if (yaw < MIN_YAW_) {
        yaw = MIN_YAW_;
    } else if (yaw > MAX_YAW_) {
        yaw = MAX_YAW_;
    }
}

void ControlYaw::publishThrustCommands() {
    std_msgs::msg::Float64 thrust_msg;

    thrust_msg.data = std::abs(calculated_thrust_);

    limitValue(thrust_msg.data);

    if (std::abs(error_) < 0.3){
        thrust_msg.data = std::abs(thrust_msg.data) * 0.3; 
    } else if ((std::abs(error_) >= 0.3) && (std::abs(error_) < 0.5)){
        thrust_msg.data = std::abs(thrust_msg.data) * 0.5;
    }

    bool estado_1 = (current_yaw_ > target_yaw_) && (current_yaw_ > 0);
    bool estado_2 = (current_yaw_ > target_yaw_) && (current_yaw_ < 0);
    bool estado_3 = (current_yaw_ == 0) && (target_yaw_ < 0);

    bool estado_4 = (current_yaw_ < target_yaw_) && (current_yaw_ > 0);
    bool estado_5 = (current_yaw_ < target_yaw_) && (current_yaw_ < 0) ;
    bool estado_6 = (current_yaw_ == 0) && (target_yaw_ > 0);

    if  (estado_1 || estado_2 || estado_3){
        // Velocidad para propulsor derecho
        propulsor_publishers_[1]->publish(thrust_msg);

        // Velocidad para propulsor izquierdo
        thrust_msg.data = -thrust_msg.data; // inventir el giro para el giro
        propulsor_publishers_[0]->publish(thrust_msg);
    } else if (estado_4 || estado_5 || estado_6){
        // Velocidad para propulsor izquierdo
        propulsor_publishers_[0]->publish(thrust_msg);

        // Velocidad para propulsor derecho
        thrust_msg.data = -thrust_msg.data; // inventir el giro para el giro
        propulsor_publishers_[1]->publish(thrust_msg);
    }

}

} // namespace auv_control_yaw