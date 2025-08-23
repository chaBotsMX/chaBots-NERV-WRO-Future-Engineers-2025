#ifndef TEENSY_COMM_NODE_H
#define TEENSY_COMM_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <chrono>
#include <atomic>
#include <array>
#include <mutex>
#include <cmath>
#include <limits>
#include <vector>
#include <cstdint>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <algorithm>

using namespace std::chrono_literals;

class TeensyCommNode : public rclcpp::Node {
public:
  TeensyCommNode();

private:
  // Serial
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr control_sub_;

  // Estado
  std::atomic<float> angle_deg_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> pwm_{70.0f};
  std::atomic<float> kp_{2.0f};
  std::atomic<float> heading{std::numeric_limits<float>::quiet_NaN()};
  std::vector<ClusterProps> clusters_;

  // Subscripción odometría
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Callback para la odometría
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  static inline float pointAngFromRobot(float iteration, float angleInc, float minAngle);
  static inline float pointAngX(float ang, float r){ return std::cos(ang) * r; }
  static inline float pointAngY(float ang, float r){ return std::sin(ang) * r; }

  static inline float getDiffAngle(float ang1, float r1, float ang2, float r2);
  static inline float getEuclideanDistance(float ang1, float r1, float ang2, float r2);
  static float angulo_positivo(float a);

  static std::array<uint8_t, 6> empaquetar(uint16_t ang_tenths, uint8_t pwm_byte, uint8_t kp_byte);

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_timer();
}

#endif
