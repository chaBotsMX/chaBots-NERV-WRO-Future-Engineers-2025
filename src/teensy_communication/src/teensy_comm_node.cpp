#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>
#include <chrono>
#include <atomic>
#include <array>
#include <mutex>
#include <cmath>
#include <limits>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <algorithm>

using namespace std::chrono_literals;

class SerialPort {
public:
  SerialPort() : fd_(-1) {}
  ~SerialPort() { close_port(); }

  bool open_port(const std::string &port, int baudrate) {
    close_port();
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) { last_error_ = std::string("open() failed: ") + std::strerror(errno); return false; }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) { last_error_ = std::string("tcgetattr() failed: ")+ std::strerror(errno); close_port(); return false; }
    cfmakeraw(&tio);

    speed_t speed = B115200;
    switch (baudrate) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default: speed = B115200; break;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB; // 8N1
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) { last_error_ = std::string("tcsetattr() failed: ") + std::strerror(errno); close_port(); return false; }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close_port() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
  bool is_open() const { return fd_ >= 0; }

  bool write_bytes(const uint8_t* data, size_t len) {
    if (fd_ < 0) return false;
    size_t total = 0;
    while (total < len) {
      ssize_t n = ::write(fd_, data + total, len - total);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) { usleep(1000); continue; }
        last_error_ = std::string("write() failed: ") + std::strerror(errno);
        return false;
      }
      total += static_cast<size_t>(n);
    }
    return true;
  }

  const std::string& last_error() const { return last_error_; }

private:
  int fd_;
  std::string last_error_;
};

class TeensyCommNode : public rclcpp::Node {
public:
  TeensyCommNode() : Node("teensy_comm") {
    // Parámetros
    port_ = declare_parameter<std::string>("port", "/dev/ttyAMA0");
    baud_ = declare_parameter<int>("baud", 115200);
    rate_hz_ = declare_parameter<int>("rate_hz", 50);
    send_mode_ = declare_parameter<std::string>("send_mode", "scan_vec_angle"); // 'constant_one'|'angle'|'scan_vec_angle'
    angle_topic_ = declare_parameter<std::string>("angle_topic", "/desired_angle_deg");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");

    // Serial
    RCLCPP_INFO(get_logger(), "Opening serial: %s @ %d", port_.c_str(), baud_);
    if (!serial_.open_port(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "Serial open failed: %s", serial_.last_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial opened.");
    }

    // Suscripciones
    angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      angle_topic_, 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg){ last_angle_deg_.store(msg->data); }
    );

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TeensyCommNode::on_scan, this, std::placeholders::_1)
    );

    // Timer de envío
    const int period_ms = std::max(1, 1000 / std::max(1, rate_hz_));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&TeensyCommNode::on_timer, this));
  }

private:
  // Empaquetado: [0xAA][HIGH][LOW][CHK], CHK = XOR(0xAA ^ HIGH ^ LOW)
  static std::array<uint8_t, 4> make_frame_u16(uint16_t value) {
    uint8_t hi = static_cast<uint8_t>((value >> 8) & 0xFF);
    uint8_t lo = static_cast<uint8_t>(value & 0xFF);
    uint8_t chk = static_cast<uint8_t>(0xAA ^ hi ^ lo);
    return {0xAA, hi, lo, chk};
  }

  static uint16_t clamp_u16(int v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return static_cast<uint16_t>(v);
  }

  static float wrap_angle_rad(float a) {
    // Devuelve a en [0, 2π)
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    while (a >= two_pi) a -= two_pi;
    while (a < 0.0f) a += two_pi;
    return a;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Sumar vectores (r*cosθ, r*sinθ) para θ en [0°, 180°] = [0, π]
    const float a_min = msg->angle_min;       // rad
    const float a_inc = msg->angle_increment; // rad/step
    const float zero  = 0.0f;
    const float pi    = static_cast<float>(M_PI);

    double sum_x = 0.0;
    double sum_y = 0.0;
    size_t used  = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float ang = a_min + a_inc * static_cast<float>(i);
      if (ang < zero || ang > pi) continue; // solo 0..180°

      float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;

      sum_x += static_cast<double>(r * std::cos(ang));
      sum_y += static_cast<double>(r * std::sin(ang));
      ++used;
    }

    // Resultado: ángulo del vector resultante
    float angle_res_rad = 0.0f;
    if (used > 0) {
      angle_res_rad = static_cast<float>(std::atan2(sum_y, sum_x)); // [-π, π]
      angle_res_rad = wrap_angle_rad(angle_res_rad);                 // [0, 2π)
    } else {
      angle_res_rad = std::numeric_limits<float>::quiet_NaN();
    }

    float angle_res_deg = angle_res_rad * 180.0f / static_cast<float>(M_PI);

    // Guarda para enviar por serie si se pide ese modo
    last_vec_angle_deg_.store(angle_res_deg);

    // Log de depuración
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,  // cada 500 ms
      "VEC SUM 0..180° -> used=%zu  sum_x=%.3f  sum_y=%.3f  angle=%.3f rad (%.2f deg)",
      used, sum_x, sum_y, angle_res_rad, angle_res_deg
    );
  }

  void on_timer() {
    if (!serial_.is_open()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Serial not open, skipping write...");
      return;
    }

    uint16_t value_to_send = 1; // default

    if (send_mode_ == "angle") {
      float a = last_angle_deg_.load();
      while (a >= 360.0f) a -= 360.0f;
      while (a < 0.0f) a += 360.0f;
      int tenths = static_cast<int>(a * 10.0f + 0.5f); // 0..3600
      value_to_send = clamp_u16(tenths);

    } else if (send_mode_ == "scan_vec_angle") {
      float deg = last_vec_angle_deg_.load();           // puede ser NaN si no hay datos
      if (!std::isfinite(deg)) {
        value_to_send = 0; // o 65535 como inválido, si prefieres
      } else {
        while (deg >= 360.0f) deg -= 360.0f;
        while (deg < 0.0f)    deg += 360.0f;
        int tenths = static_cast<int>(deg * 10.0f + 0.5f); // décimas de grado
        value_to_send = clamp_u16(tenths);
      }
    } else {
      // constant_one u otros -> 1
      value_to_send = 1;
    }

    auto frame = make_frame_u16(value_to_send);
    if (!serial_.write_bytes(frame.data(), frame.size())) {
      RCLCPP_WARN(get_logger(), "Serial write failed: %s", serial_.last_error().c_str());
    }
  }

  // Parámetros
  std::string port_;
  int baud_;
  int rate_hz_;
  std::string send_mode_;
  std::string angle_topic_;
  std::string scan_topic_;

  // Serial
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Estado
  std::atomic<float> last_angle_deg_{0.0f};
  std::atomic<float> last_vec_angle_deg_{std::numeric_limits<float>::quiet_NaN()};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}
