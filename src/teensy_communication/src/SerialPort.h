#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

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

struct ClusterProps {
  float angle;     // rad: dirección (punto inicio -> punto fin)
  float magnitud;  // distancia euclidiana inicio-fin
  float size;      // número de muestras abarcadas (incluye gaps tolerados si los agregas)
  int initPoint; // índice de inicio en ranges
};

class SerialPort {
public:
  SerialPort() : puerto_(-1) {}
  ~SerialPort() { close_port(); }

  bool open_port();
  void close_port();
  bool is_open() const { return puerto_ >= 0; }

  bool write_bytes(const uint8_t* data, size_t len);

  std::string last_error() const { return last_error_; }

private:
  int puerto_;
  std::string last_error_;
};

#endif
