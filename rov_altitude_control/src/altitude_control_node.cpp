#include "rclcpp/rclcpp.hpp"
#include "rov_altitude_control/altitude_controller.hpp"
#include "rov_altitude_control/longitud_controller.hpp"
#include <nav_msgs/msg/odometry.hpp>
//#include <std_srvs/srv/set_float64.hpp>

class AltitudeControlNode : public rclcpp::Node {
public:
    AltitudeControlNode() : Node("altitude_control_node") {
        // Dejar el constructor vacío o solo para inicializaciones básicas
    }

    void init() {
        altitude_controller_ = std::make_shared<rov_altitude_control::AltitudeController>(shared_from_this());
        longitud_controller_ = std::make_shared<rov_longitud_control::LongitudController>(shared_from_this());

        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/rov_max/odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                altitude_controller_->updateControl(msg);
                longitud_controller_->updateControl(msg);
            });

        altitude_controller_->setTargetAltitude(-0.6);
        longitud_controller_->setTargetLongitud(-5.0);

    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    std::shared_ptr<rov_altitude_control::AltitudeController> altitude_controller_;
    std::shared_ptr<rov_longitud_control::LongitudController> longitud_controller_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AltitudeControlNode>();
    node->init();  // Llama a la función init después de la creación del nodo
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
