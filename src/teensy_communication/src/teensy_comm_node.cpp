#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector2.hpp>

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
    // Parámetros constantes
    port_        = "/dev/ttyAMA0";
    baud_        = 115200;
    rate_hz_     = 200;
    send_mode_   = "scan_vec_angle"; // 'constant_one'|'angle'|'scan_vec_angle'
    angle_topic_ = "/desired_angle_deg";

    // Serial
    RCLCPP_INFO(get_logger(), "Opening serial: %s @ %d", port_.c_str(), baud_);
    if (!serial_.open_port(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "Serial open failed: %s", serial_.last_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial opened.");
    }

    // Publisher: ángulo resultante (deg) para Foxglove
    angle_pub_ = create_publisher<std_msgs::msg::Float32>("/scan_vec_angle_deg", 10);

    // Suscripciones
    angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      angle_topic_, 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg){ last_angle_deg_.store(msg->data); }
    );

    // Un solo tópico para vel y kp: geometry_msgs/Vector2 (x=vel_mps, y=kp)
    control_sub_ = create_subscription<geometry_msgs::msg::Vector2>(
      "/control_params", 10,
      [this](const geometry_msgs::msg::Vector2::SharedPtr msg){
        last_vel_mps_.store(static_cast<float>(msg->x));
        last_kp_.store(static_cast<float>(msg->y));
      }
    );

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&TeensyCommNode::on_scan, this, std::placeholders::_1)
    );

    // Timer de envío
    const int period_ms = std::max(1, 1000 / std::max(1, rate_hz_));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&TeensyCommNode::on_timer, this));
  }

private:
  // Frame V2: [0xAB][ANG_H][ANG_L][VEL_H][VEL_L][KP_H][KP_L][CHK]
  static std::array<uint8_t, 8> make_frame_v2(uint16_t ang_tenths, uint16_t vel_mmps, uint16_t kp_milli) {
    std::array<uint8_t, 8> f{
      0xAB,
      static_cast<uint8_t>((ang_tenths >> 8) & 0xFF),
      static_cast<uint8_t>(ang_tenths & 0xFF),
      static_cast<uint8_t>((vel_mmps >> 8) & 0xFF),
      static_cast<uint8_t>(vel_mmps & 0xFF),
      static_cast<uint8_t>((kp_milli >> 8) & 0xFF),
      static_cast<uint8_t>(kp_milli & 0xFF),
      0x00  // CHK placeholder
    };
    uint8_t chk = 0x00;
    for (size_t i = 0; i < 7; ++i) chk ^= f[i];
    f[7] = chk;
    return f;
  }

  static uint16_t clamp_u16(int v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return static_cast<uint16_t>(v);
  }

  static float wrap_angle_rad(float a) {
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    while (a >= two_pi) a -= two_pi;
    while (a < 0.0f) a += two_pi;
    return a;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Suma vectorial de 0..180° (0..π rad)
    const float a_min = msg->angle_min;
    const float a_inc = msg->angle_increment;
    const float zero  = 0.0f;
    const float pi    = static_cast<float>(M_PI);

    double sum_x = 0.0;
    double sum_y = 0.0;
    size_t used  = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float ang = a_min + a_inc * static_cast<float>(i);
      if (ang < zero || ang > pi) continue;

      float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;

      sum_x += static_cast<double>(r * std::cos(ang));
      sum_y += static_cast<double>(r * std::sin(ang));
      ++used;
    }

    float angle_res_rad = std::numeric_limits<float>::quiet_NaN();
    if (used > 0) {
      angle_res_rad = static_cast<float>(std::atan2(sum_y, sum_x)); // [-π, π]
      angle_res_rad = wrap_angle_rad(angle_res_rad);                 // [0, 2π)
    }
    float angle_res_deg = angle_res_rad * 180.0f / static_cast<float>(M_PI);

    // Publica para Foxglove
    if (std::isfinite(angle_res_deg)) {
      std_msgs::msg::Float32 m;
      m.data = angle_res_deg;
      angle_pub_->publish(m);
    }

    // Guarda para UART
    last_vec_angle_deg_.store(angle_res_deg);
  }

  void on_timer() {
    if (!serial_.is_open()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Serial not open, skipping write...");
      return;
    }

    // Ángulo a enviar
    float deg = (send_mode_ == "angle") ? last_angle_deg_.load() : last_vec_angle_deg_.load();
    uint16_t ang_tenths = 0;
    if (std::isfinite(deg)) {
      while (deg >= 360.0f) deg -= 360.0f;
      while (deg < 0.0f)    deg += 360.0f;
      ang_tenths = clamp_u16(static_cast<int>(deg * 10.0f + 0.5f));
    } else {
      ang_tenths = 0; // o 65535 para inválido
    }

    // Control (persisten último valor recibido)
    float vel_mps = last_vel_mps_.load(); // default 0.0
    float kp      = last_kp_.load();      // default 0.0

    // Escalados UART
    uint16_t vel_mmps = clamp_u16(static_cast<int>(vel_mps * 1000.0f + 0.5f));  // m/s -> mm/s
    uint16_t kp_milli = clamp_u16(static_cast<int>(kp * 1000.0f + 0.5f));       // kp con 3 decimales

    auto frame = make_frame_v2(ang_tenths, vel_mmps, kp_milli);
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

  // Serial
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector2>::SharedPtr control_sub_;

  // Estado (persisten entre mensajes)
  std::atomic<float> last_angle_deg_{0.0f};                               // modo "angle"
  std::atomic<float> last_vec_angle_deg_{std::numeric_limits<float>::quiet_NaN()}; // ángulo LIDAR
  std::atomic<float> last_vel_mps_{0.0f};                                 // velocidad m/s
  std::atomic<float> last_kp_{0.0f};                                      // kp
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}
