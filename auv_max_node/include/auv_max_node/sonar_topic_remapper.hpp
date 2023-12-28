#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LaserScanFrameRemapper : public rclcpp::Node {
public:
    LaserScanFrameRemapper();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointCloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointCloud_;
};
