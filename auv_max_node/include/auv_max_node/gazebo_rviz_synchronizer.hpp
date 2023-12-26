#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

class OdometryToTFPublisher : public rclcpp::Node {
public:
    OdometryToTFPublisher();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
