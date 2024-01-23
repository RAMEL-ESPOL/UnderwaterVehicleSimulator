#include "auv_max_node/cmd_vel_thrust_converter.hpp"
#include "auv_max_node/sonar_topic_remapper.hpp"
#include "auv_max_node/gazebo_rviz_synchronizer.hpp"
#include "auv_max_node/imu_topic_remapper.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Creación e inicialización de los nodos
    auto cmdVelThrustConverterNode = std::make_shared<CmdVelThrustConverter>();
    cmdVelThrustConverterNode->init();

    auto laserScanFrameRemapperNode = std::make_shared<LaserScanFrameRemapper>();

    auto odometryToTFPublisherNode = std::make_shared<OdometryToTFPublisher>();

    auto imuFrameRemapperNode = std::make_shared<IMUFrameRemapper>();

    // Ejecución simultánea de los nodos
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(cmdVelThrustConverterNode);
    executor.add_node(laserScanFrameRemapperNode);
    executor.add_node(odometryToTFPublisherNode);
    executor.add_node(imuFrameRemapperNode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
