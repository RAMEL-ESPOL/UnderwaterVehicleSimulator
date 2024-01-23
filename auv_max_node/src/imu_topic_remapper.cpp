#include "auv_max_node/imu_topic_remapper.hpp"

IMUFrameRemapper::IMUFrameRemapper() : Node("imu_frame_remapper") {
    if(!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Node para arreglar el frame_id del IMU no inicializado!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Node para arreglar el frame_id del IMU inicializado!");

    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/model/auv_max/imu", 
        10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            auto new_msg = *msg;
            new_msg.header.stamp = this->now();
            new_msg.header.frame_id = "auv_max_shell"; // Nuevo frame_id
            publisher_->publish(new_msg);
        }
    );

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/model/auv_max/remapped_imu",
        10
    );
}
