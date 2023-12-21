#include "auv_max_sonar/sonar_obstacle_detector.hpp"
#include <iostream>

SonarObstacleDetector::SonarObstacleDetector()
: Node("sonar_obstacle_detector") {
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/model/auv_max/sonar", 10,
      std::bind(&SonarObstacleDetector::sonarCallback, this, std::placeholders::_1));
}

void SonarObstacleDetector::sonarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Umbral de distancia en metros para la detección de obstáculos
  float distance_threshold = 2.0; 

  // Dividir el rango del sonar en 5 segmentos
  int num_readings = msg->ranges.size();
  RCLCPP_INFO(this->get_logger(), "Número de lecturas: %d", num_readings);
  int segment_size = num_readings / 5;
  std::vector<bool> obstacles_detected(5, false); // Derecha, Diagonal Derecha, Frente, Diagonal Izquierda, Izquierda

  for (int i = 0; i < num_readings; ++i) {
    if (msg->ranges[i] <= distance_threshold) {
      int segment = i / segment_size;
      obstacles_detected[segment] = true;
    }
  }

  // Reportar la detección de obstáculos por segmento
  if (obstacles_detected[4]) {
    RCLCPP_INFO(this->get_logger(), "Obstáculo detectado a la izquierda!");
  }
  if (obstacles_detected[3]) {
    RCLCPP_INFO(this->get_logger(), "Obstáculo detectado en diagonal izquierda!");
  }
  if (obstacles_detected[2]) {
    RCLCPP_INFO(this->get_logger(), "Obstáculo detectado al frente!");
  }
  if (obstacles_detected[1]) {
    RCLCPP_INFO(this->get_logger(), "Obstáculo detectado en diagonal derecha!");
  }
  if (obstacles_detected[0]) {
    RCLCPP_INFO(this->get_logger(), "Obstáculo detectado a la derecha!");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
