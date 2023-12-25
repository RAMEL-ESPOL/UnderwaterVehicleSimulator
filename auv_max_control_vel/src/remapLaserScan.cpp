#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanFrameRemapper : public rclcpp::Node {
public:
    LaserScanFrameRemapper() : Node("laser_scan_frame_remapper") {
        // Suscriptor al tópico original de LaserScan
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

        // Publicador en el nuevo tópico con el frame_id modificado
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/model/auv_max/remapped_sonar",
            10
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanFrameRemapper>());
    rclcpp::shutdown();
    return 0;
}
