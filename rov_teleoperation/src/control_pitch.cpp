#include "rov_teleoperation/control_pitch.hpp"
#include <iostream>
#include <fstream>

namespace rov_control_pitch {

ControlPitch::ControlPitch(rclcpp::Node::SharedPtr node)
    : node_(node), target_pitch_(0.0), current_pitch_(0.0),
      error_(0.0), prev_error_(0.0), integral_(0.0), derivative_(0.0),
      kp_(20.0), ki_(0.04), kd_(18.0), calculated_thrust_(0.0) {
    initializePublishers();
}

void ControlPitch::setTargetPitch(double pitch) {
    target_pitch_ = pitch;
    limitPitch(target_pitch_);
}

void ControlPitch::updateControl(const nav_msgs::msg::Odometry::SharedPtr& odometry_msg) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(odometry_msg->pose.pose.orientation, quaternion);

    // Convertir a ángulos de Euler
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quaternion);
    m.getRPY(roll, pitch, yaw);

    pitch = std::round(pitch * 100.0) / 100.0;

    current_pitch_ = pitch;   

    error_ = target_pitch_ - current_pitch_;
    integral_ += error_;
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;

    std_msgs::msg::Float64 target_pitch_msg;
    target_pitch_msg.data = target_pitch_;
    pub_target_pitch_->publish(target_pitch_msg);

    computePID();
    publishThrustCommands();
}

void ControlPitch::initializePublishers() {
    propulsor_publishers_[0] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_left/cmd_thrust", 10);
    propulsor_publishers_[1] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_vert_thrust_right/cmd_thrust", 10);
    propulsor_publishers_[2] = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/joint/shell_to_center_thrust/cmd_thrust", 10);
    
    pub_target_pitch_ = node_->create_publisher<std_msgs::msg::Float64>("/model/rov_max/longitud_controller/target_pitch", 10);
}

void ControlPitch::computePID() {
    double thrust = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
    // Normalizar o limitar el valor de 'thrust' si es necesario
    calculated_thrust_ = thrust;
}

void ControlPitch::limitValue(double& value, bool is_center) {
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

void ControlPitch::limitPitch(double& pitch) {
    if (pitch < MIN_PITCH_) {
        pitch = MIN_PITCH_;
    } else if (pitch > MAX_PITCH_) {
        pitch = MAX_PITCH_;
    }
}

void ControlPitch::publishThrustCommands() {
    std_msgs::msg::Float64 thrust_msg;

    thrust_msg.data = std::abs(calculated_thrust_);

    if (std::abs(error_) < 0.5){
        thrust_msg.data = std::abs(calculated_thrust_) * 0.8; 
    }

    bool estado_1 = (current_pitch_ < target_pitch_) && (current_pitch_ > 0);
    bool estado_2 = (current_pitch_ < target_pitch_) && (current_pitch_ < 0);
    bool estado_3 = (current_pitch_ == 0) && (target_pitch_ > 0);

    bool estado_4 = (current_pitch_ > target_pitch_) && (current_pitch_ > 0);
    bool estado_5 = (current_pitch_ > target_pitch_) && (current_pitch_ < 0);
    bool estado_6 = (current_pitch_ == 0) && (target_pitch_ < 0);

    // Propulsor izquierdo [0]
    // Propulsor derecho [1]
    // Propulsor central [2]

    if (estado_1 || estado_2 || estado_3) {
        limitValue(thrust_msg.data, true);
        propulsor_publishers_[2]->publish(thrust_msg);

        limitValue(thrust_msg.data);
        thrust_msg.data = -thrust_msg.data; // inventir el giro para el giro
        propulsor_publishers_[0]->publish(thrust_msg);
        propulsor_publishers_[1]->publish(thrust_msg);
    } else if (estado_4 || estado_5 || estado_6) {
        limitValue(thrust_msg.data, true);
        thrust_msg.data = -thrust_msg.data; // inventir el giro para el giro
        propulsor_publishers_[2]->publish(thrust_msg);

        thrust_msg.data = std::abs(thrust_msg.data);
        limitValue(thrust_msg.data);
        propulsor_publishers_[0]->publish(thrust_msg);
        propulsor_publishers_[1]->publish(thrust_msg);
    }
}

} // namespace rov_control_pitch