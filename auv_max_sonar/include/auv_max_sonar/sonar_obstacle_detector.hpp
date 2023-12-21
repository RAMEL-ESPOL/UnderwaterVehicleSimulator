#ifndef SONAR_OBSTACLE_DETECTOR_H
#define SONAR_OBSTACLE_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

class SonarObstacleDetector : public rclcpp::Node {
public:
  SonarObstacleDetector();

private:
  void sonarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

#endif // SONAR_OBSTACLE_DETECTOR_H