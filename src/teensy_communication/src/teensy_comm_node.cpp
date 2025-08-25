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


class SerialPort {
public:
  SerialPort() : puerto_(-1) {}
  ~SerialPort() { close_port(); }

  bool open_port() {
    close_port();
    puerto_ = ::open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (puerto_ < 0) {
      last_error_ = std::string("open() failed: ") + std::strerror(errno);
      return false;
    }

    struct termios tio{};
    if (tcgetattr(puerto_, &tio) != 0) {
      last_error_ = std::string("tcgetattr failed: ") + std::strerror(errno);
      close_port();
      return false;
    }

    cfmakeraw(&tio);
    // Usa un baud soportado por tu sistema. B2000000 (2 Mbps) suele estar disponible.
    cfsetispeed(&tio, B2000000);
    cfsetospeed(&tio, B2000000);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB; // 8N1
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(puerto_, TCSANOW, &tio) != 0) {
      last_error_ = std::string("tcsetattr failed: ") + std::strerror(errno);
      close_port();
      return false;
    }
    tcflush(puerto_, TCIOFLUSH);
    return true;
  }

  void close_port() { if (puerto_ >= 0) { ::close(puerto_); puerto_ = -1; } }
  bool is_open() const { return puerto_ >= 0; }

  bool write_bytes(const uint8_t* data, size_t len) {
    size_t total = 0;
    while (total < len) {
      ssize_t n = ::write(puerto_, data + total, len - total);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
        return false;
      }
      total += static_cast<size_t>(n);
    }
    return true;
  }

  std::string last_error() const { return last_error_; }

private:
  int puerto_;
  std::string last_error_;
};

class TeensyCommNode : public rclcpp::Node {
public:
  TeensyCommNode() : Node("teensy_comm") {
    (void)serial_.open_port();

    angle_pub_   = create_publisher<std_msgs::msg::Float32>("/send_angle", 10);
    cluster_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/clusters_markers", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),std::bind(&TeensyCommNode::on_scan, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10,std::bind(&TeensyCommNode::on_odom, this, std::placeholders::_1));
    timer_ = create_wall_timer(10ms, std::bind(&TeensyCommNode::on_timer, this));
  }

private:
  // --------- Utilidades trig/cartesianas ----------
  static inline float pointAngFromRobot(float iteration, float angleInc, float minAngle){
    return  minAngle + angleInc * iteration; // rad
  }
  static inline float pointAngX(float ang, float r){ return std::cos(ang) * r; }
  static inline float pointAngY(float ang, float r){ return std::sin(ang) * r; }
  static inline float getDiffAngle(float ang1, float r1, float ang2, float r2){
    const float dx = pointAngX(ang2, r2) - pointAngX(ang1, r1);
    const float dy = pointAngY(ang2, r2) - pointAngY(ang1, r1);
    return std::atan2(dy, dx); // rad
  }
  static inline float getEuclideanDistance(float ang1, float r1, float ang2, float r2){
    const float dx = pointAngX(ang2, r2) - pointAngX(ang1, r1);
    const float dy = pointAngY(ang2, r2) - pointAngY(ang1, r1);
    return std::hypot(dx, dy);
  }
  static float angulo_positivo(float a) {
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    while (a >= two_pi) a -= two_pi;
    while (a < 0.0f)    a += two_pi;
    return a;
  }
  
  float getOffsetFromCenter(){
      bool getleft = false;
      bool getright = false;
      int rightDis = 0;
      int leftDis = 0;
    for(int i = 0; i < msg->ranges.size(); ++i) {
      const float ang = angle_min + angle_inc * static_cast<float>(i);
      if (ang < 0.0f || ang > pi) continue; // 0..180°
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      if (getleft == false) {
        if (r < 0.78f)
          rightDis = 
        getleft = true;
      }
      if (getright == false) {
        right = ang;
        getright = true;
      }
  }

// -----------maquina de estados ----------

  void state_machine() {
    if (laps == 1) {
      getMiddleAndEnd();
    } else if (end == 1) {
      getEnd();
    } else if (safe == 1) {
      goAngleAndCenter();
    } else if (turn == 1) {
      turnRobot();
    } else if (dis == 0) {
      searchHole();
    }
    
  }
  //funciones de la maquina de estados
  void getMiddleAndEnd() {
    // Implementar lógica para obtener el punto medio y el final
  }

  void getEnd() {
    // Implementar lógica para obtener el final
  }

  void goAngleAndCenter() {

  }

  void turnRobot() {
    // Implementar lógica para girar el robot
  }

  void searchHole() {
    // Implementar lógica para buscar un agujero
  }

  //funcion para enviar por serial
  static std::array<uint8_t, 6> empaquetar(uint16_t ang_tenths, uint8_t pwm_byte, uint8_t kp_byte, rclcpp::Logger logger) {
    std::array<uint8_t, 6> f{};
    f[0] = 0xAB;
    f[1] = static_cast<uint8_t>((ang_tenths >> 8) & 0xFF);
    f[2] = static_cast<uint8_t>(ang_tenths & 0xFF);
    f[3] = pwm_byte;
    f[4] = kp_byte;
    uint8_t chk = 0x00;
  //  RCLCPP_INFO(logger, "angulo mandado: %ld", ang_tenths);
    for (size_t i = 0; i < 5; ++i) chk ^= f[i];
    f[5] = chk;
    return f;
  }


  //callbacks que se ejecutan al leer un topic
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const float angle_min = msg->angle_min;
    const float angle_inc = msg->angle_increment;
    const float pi        = static_cast<float>(M_PI);


    double sum_x = 0.0, sum_y = 0.0;
    size_t used = 0;


    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float ang = angle_min + angle_inc * static_cast<float>(i);
      if (ang < 0.0f || ang > pi) continue; // 0..180°
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      sum_x += static_cast<double>(r * std::cos(ang));
      sum_y += static_cast<double>(r * std::sin(ang));
      ++used;
    }

    float angle_deg = std::numeric_limits<float>::quiet_NaN();
    if (used > 0) {
      float angle = static_cast<float>(std::atan2(sum_y, sum_x));
      angle = angulo_positivo(angle);
      angle_deg = angle * 180.0f / static_cast<float>(M_PI);
    }
    angle_deg_.store(angle_deg);
  }

  void on_timer() {

    float deg = angle_deg_.load();
    float head = heading.load();
    int vel = 50;
    if (std::fabs(head) > 1020){
      vel = 1;
    }
    //RCLCPP_INFO(this->get_logger(), "angulo mandado: %f", deg);
    auto frame = empaquetar(static_cast<uint16_t>(deg), vel, 10,this->get_logger());
    (void)serial_.write_bytes(frame.data(), frame.size());
  }
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto& q = msg->pose.pose.orientation;
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0f / static_cast<float>(M_PI);
    if (yaw_deg >= 180.0f) yaw_deg -= 360.0f;
    if (yaw_deg <  -180.0f) yaw_deg += 360.0f;

    static bool init=false; static float prev=0.0f; static float acc=0.0f;
    if (!init) { prev = yaw_deg; acc = 0.0f; init = true; }
    else {
      float d = yaw_deg - prev;
      if (d >  180.0f) d -= 360.0f;
      if (d < -180.0f) d += 360.0f;
      acc += d; prev = yaw_deg;
    }
    heading.store(acc);
    heading360_.store(wrap_360(acc));
  }
  // Serial
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr                                    timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            angle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // variables persistentes
  std::atomic<float> heading360{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> angle_deg_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> pwm_{70.0f};
  std::atomic<float> kp_{2.0f};
  std::atomic<float> heading{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<bool> dis{false};
  std::atomic<bool> turn{false};
  std::atomic<bool> safe{true};
  std::atomic<bool> end{false};
  std::atomic<bool> laps{false};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}
