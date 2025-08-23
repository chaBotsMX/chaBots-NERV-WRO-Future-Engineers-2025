#include <rclcpp/rclcpp.hpp>

#include "SerialPort.h"
#include "TeensyCommNode.h"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}
