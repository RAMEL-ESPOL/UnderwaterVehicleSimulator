#include <urdf/model.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  if (argc != 2){
    RCLCPP_ERROR(rclcpp::get_logger("my_parser"), "Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    RCLCPP_ERROR(rclcpp::get_logger("my_parser"), "Failed to parse urdf file");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("my_parser"), "Successfully parsed urdf file");
  return 0;
}
