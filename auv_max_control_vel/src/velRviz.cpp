#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdometryToTFPublisher : public rclcpp::Node {
public:
    OdometryToTFPublisher() : Node("odometry_to_tf_publisher") {
        // Suscripción al tópico de odometría
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/auv_max/odometry", 10, 
            std::bind(&OdometryToTFPublisher::odomCallback, this, std::placeholders::_1));

        // Inicialización del publicador TF
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Crear y llenar el mensaje de transformación
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

        // Publicar la transformación
        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryToTFPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}