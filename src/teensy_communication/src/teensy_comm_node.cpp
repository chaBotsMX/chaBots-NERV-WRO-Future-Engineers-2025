#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <string>
#include <chrono>
#include <atomic>
#include <array>
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
    if (fd_ < 0) {
      last_error_ = "open() failed: " + std::string(std::strerror(errno));
      return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      last_error_ = "tcgetattr() failed: " + std::string(std::strerror(errno));
      close_port();
      return false;
    }

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

    // lectura no bloqueante
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      last_error_ = "tcsetattr() failed: " + std::string(std::strerror(errno));
      close_port();
      return false;
    }

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
        last_error_ = "write() failed: " + std::string(std::strerror(errno));
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
    port_ = declare_parameter<std::string>("port", "/dev/ttyAMA0");
    baud_ = declare_parameter<int>("baud", 115200);
    rate_hz_ = declare_parameter<int>("rate_hz", 50);
    send_mode_ = declare_parameter<std::string>("send_mode", "constant_one"); // 'constant_one' | 'angle'
    angle_topic_ = declare_parameter<std::string>("angle_topic", "/desired_angle_deg");

    RCLCPP_INFO(get_logger(), "Opening serial: %s @ %d", port_.c_str(), baud_);
    if (!serial_.open_port(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "Serial open failed: %s", serial_.last_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial opened.");
    }

    if (send_mode_ == "angle") {
      angle_sub_ = create_subscription<std_msgs::msg::Float32>(
        angle_topic_, 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg){
          last_angle_deg_.store(msg->data);
        }
      );
    }

    int period_ms = std::max(1, 1000 / std::max(1, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&TeensyCommNode::on_timer, this)
    );
  }

private:
  // Frame: [0xAA][HIGH][LOW][CHK], CHK = XOR(0xAA ^ HIGH ^ LOW)
  std::array<uint8_t, 4> make_frame_u16(uint16_t value) {
    uint8_t high = static_cast<uint8_t>((value >> 8) & 0xFF);
    uint8_t low  = static_cast<uint8_t>(value & 0xFF);
    uint8_t chk  = static_cast<uint8_t>(0xAA ^ high ^ low);
    return {0xAA, high, low, chk};
  }

  static uint16_t clamp_u16(int v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return static_cast<uint16_t>(v);
  }

  void on_timer() {
    if (!serial_.is_open()) {
      // CORREGIDO: usar macro válida de “throttle”
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Serial not open, skipping write...");
      return;
    }

    uint16_t value_to_send = 1; // default: prueba

    if (send_mode_ == "angle") {
      float a = last_angle_deg_.load();
      while (a >= 360.0f) a -= 360.0f;
      while (a < 0.0f) a += 360.0f;
      int tenths = static_cast<int>(a * 10.0f + 0.5f); // 0..3600
      value_to_send = clamp_u16(tenths);
    }

    auto frame = make_frame_u16(value_to_send);
    if (!serial_.write_bytes(frame.data(), frame.size())) {
      RCLCPP_WARN(get_logger(), "Serial write failed: %s", serial_.last_error().c_str());
    }
  }

  // params
  std::string port_;
  int baud_;
  int rate_hz_;
  std::string send_mode_;
  std::string angle_topic_;

  // serial
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;

  // state
  std::atomic<float> last_angle_deg_{0.0f};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeensyCommNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
