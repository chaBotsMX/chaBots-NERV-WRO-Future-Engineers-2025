#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>

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
  SerialPort() : puerto(-1) {}
  ~SerialPort() { close_port(); }

  bool open_port() {
    close_port();
    puerto = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (puerto < 0) {
      last_error_ = std::string("open() failed: ") + std::strerror(errno);
      return false;
    }
    tcgetattr(puerto, &termios);
    cfmakeraw(&termios);
    cfsetispeed(&termios, B200000);
    cfsetospeed(&termios, B200000);

    termios.c_cflag |= (CLOCAL | CREAD);
    termios.c_cflag &= ~PARENB; // 8N1
    termios.c_cflag &= ~CSTOPB;
    termios.c_cflag &= ~CSIZE;
    termios.c_cflag |= CS8;
    termios.c_cc[VMIN]  = 0;
    termios.c_cc[VTIME] = 0;

    tcsetattr(puerto, TCSANOW, &termios);
    tcflush(puerto, TCIOFLUSH);
    return true;
  }

  void close_port() { if (puerto >= 0) { ::close(puerto); puerto = -1; } }
  bool is_open() const { return puerto >= 0; }

  void write_bytes(const uint8_t* data, size_t len) {
    size_t total = 0;
    while (total < len) {
      ssize_t n = ::write(puerto, data + total, len - total);
      total += static_cast<size_t>(n);
    }
  }

  
private:
  int puerto;
};

class TeensyCommNode : public rclcpp::Node {
public:
  TeensyCommNode() : Node("teensy_comm") {
    // Serial
    serial.open_port();
    // Publisher: ángulo resultante (deg) para Foxglove
    angle_pub_ = create_publisher<std_msgs::msg::Float32>("/send_angle", 10);
    // Un solo tópico para vel y kp: geometry_msgs/Vector3 (x=vel_mps, y=kp)
    control_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/control_params", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
        pwm.store(static_cast<float>(msg->x));
        kp.store(static_cast<float>(msg->y));
      }
    );

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&TeensyCommNode::on_scan, this, std::placeholders::_1)
    );
    
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&TeensyCommNode::on_timer, this));
  }

private:
  // Frame V2: [0xAB][ANG_H][ANG_L][VEL_H][VEL_L][KP_H][KP_L][CHK]
  static std::array<uint8_t, 8> empaquetar(uint16_t ang_tenths, uint8_t pwm_byte, uint8_t kp_byte) {
    std::array<uint8_t, 6> f{
      0xAB,
      static_cast<int>((ang_tenths >> 8) & 0xFF),
      static_cast<int>(ang_tenths & 0xFF),
      static_cast<int>(pwm_byte),
      static_cast<int>(kp_byte),
      0x00  // CHK placeholder
    };
    uint8_t check = 0x00;
    for (size_t i = 0; i < 7; ++i) check ^= f[i];
    f[7] = check;
    return f;
  }

  static float angulo_positivo(float a) {
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    while (a >= two_pi) a -= two_pi;
    while (a < 0.0f) a += two_pi;
    return a;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Suma vectorial de 0..180° (0..π rad)
    const float angle_min = msg->angle_min;
    const float angle_inc = msg->angle_increment;
    const float pi    = static_cast<float>(M_PI);

    double sum_x = 0.0;
    double sum_y = 0.0;    size_t used  = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float ang = angle_min + angle_inc * static_cast<float>(i);
      if (ang < 0 || ang > pi) continue;
      float magnitud = msg->ranges[i];
      if (!std::isfinite(magnitud) || magnitud < msg->range_min || magnitud > msg->range_max) continue;
      sum_x += static_cast<double>(magnitud * std::cos(ang));
      sum_y += static_cast<double>(magnitud * std::sin(ang));
      ++used;
    }

    float angle = std::numeric_limits<float>::quiet_NaN();

    if (used > 0) {
      angle = static_cast<float>(std::atan2(sum_y, sum_x)); // [-π, π]
      angle= angulo_positivo(angle);                 // [0, 2π)
    }
    angle = angle * 180.0f / static_cast<float>(M_PI);

    // Publica para Foxglove
    if (std::isfinite(angle)) {
      std_msgs::msg::Float32 mensaje;
      mensaje.data = angle;
      angle_pub_->publish(mensaje);
    }
    // Guarda para UART
    last_vec_angle_deg_.store(angle);
  }
  void on_timer() {
    float deg =  angle.load();
    uint16_t ang_tenths = 0;
    if (std::isfinite(deg)) {
      while (deg >= 360.0f) deg -= 360.0f;
      while (deg < 0.0f)    deg += 360.0f;
      ang_tenths = static_cast<uint16_t>(deg * 10.0f + 0.5f);
    } else {
      ang_tenths = 300; 
    }
    int send_pwm = pwm.load(); // default 0.0
    float send_kp  = kp.load();      // default 0.0

    auto frame = empaquetar(ang_tenths, send_pwm, send_kp);
    serial.write_bytes(frame.data(), frame.size());
  }

  // Serial
  SerialPort serial;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr control_sub_;

  // Estado (persisten entre mensajes)
  std::atomic<float> angle{0.0f}; // ángulo LIDAR
  std::atomic<float> pwm{70};                                 // pwm
  std::atomic<float> kp{2.0f};                                      // kp
};


int main() {
  rclcpp::init();
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}