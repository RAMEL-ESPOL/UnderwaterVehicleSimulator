#include "auv_max_node/sonar_topic_remapper.hpp"

LaserScanFrameRemapper::LaserScanFrameRemapper() : Node("laser_scan_frame_remapper") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/model/auv_max/sonar",
        10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            auto new_msg = *msg;
            new_msg.header.stamp = this->now();
            new_msg.header.frame_id = "sonar_link"; // Nuevo frame_id
            publisher_->publish(new_msg);
        }
    );

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/model/auv_max/remapped_sonar",
        10
    );

    subscription_pointCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/model/auv_max/sonar/points",
        10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            auto new_msg = *msg;
            new_msg.header.stamp = this->now();
            new_msg.header.frame_id = "sonar_link"; // Nuevo frame_id
            publisher_pointCloud_->publish(new_msg);
        }
    );

    publisher_pointCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/model/auv_max/remapped_sonar_point_cloud",
        10
    );
}
