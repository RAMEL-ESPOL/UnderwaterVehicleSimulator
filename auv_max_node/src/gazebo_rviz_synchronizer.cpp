#include "auv_max_node/gazebo_rviz_synchronizer.hpp"

OdometryToTFPublisher::OdometryToTFPublisher() : Node("odometry_to_tf_publisher") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/model/auv_max/odometry", 10, 
        std::bind(&OdometryToTFPublisher::odomCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/model/auv_max/pose",
        10,
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            auto new_msg = *msg;
            new_msg.header.stamp = this->now();
            new_msg.header.frame_id = "auv_max_shell"; // Nuevo frame_id
            RCLCPP_INFO(this->get_logger(), "PoseArray received and remapped");
            pub_pose_->publish(new_msg);
        }
    );

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/model/auv_max/remapped_pose",
        10
    );
}

void OdometryToTFPublisher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "auv_max_frame";

    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;

    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(transformStamped);
}
