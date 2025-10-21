#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <thread>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cerrno>
#include <functional>
#include <algorithm>


using namespace std::chrono_literals;

#include "TeensyObsNode.h"
#include "SerialPort.h"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::this_thread::sleep_for(std::chrono::seconds(10));
  rclcpp::spin(std::make_shared<TeensyObsNode>());
  rclcpp::shutdown();
  return 0;
}  