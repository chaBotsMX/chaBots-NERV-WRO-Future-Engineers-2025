#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

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
  int   initPoint; // índice de inicio en ranges
};

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

    control_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/control_params", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
        pwm_.store(static_cast<float>(msg->x));
        kp_.store(static_cast<float>(msg->y));
      }
    );

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&TeensyCommNode::on_scan, this, std::placeholders::_1)
    );

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

  // ---------- Empaquetado (6 bytes) ----------
  // [0xAB][ANG_H][ANG_L][PWM][KP][CHK]
  static std::array<uint8_t, 6> empaquetar(uint16_t ang_tenths, uint8_t pwm_byte, uint8_t kp_byte) {
    std::array<uint8_t, 6> f{};
    f[0] = 0xAB;
    f[1] = static_cast<uint8_t>((ang_tenths >> 8) & 0xFF);
    f[2] = static_cast<uint8_t>(ang_tenths & 0xFF);
    f[3] = pwm_byte;
    f[4] = kp_byte;
    uint8_t chk = 0x00;
    for (size_t i = 0; i < 5; ++i) chk ^= f[i];
    f[5] = chk;
    return f;
  }

  // ---------- Procesamiento principal ----------
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const float angle_min = msg->angle_min;
    const float angle_inc = msg->angle_increment;
    const float pi        = static_cast<float>(M_PI);

    // (A) Ángulo global (vectorial) para publicar como Float32
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

    // (B) Detección súper simple de clústers (segmento recto)
    clusters_.clear();
    for (int j = 0; j + 1 < static_cast<int>(msg->ranges.size()); ++j) {
      const float r0 = msg->ranges[j];
      const float r1 = msg->ranges[j+1];
      if (!std::isfinite(r0) || r0 < msg->range_min || r0 > msg->range_max) continue;
      if (!std::isfinite(r1) || r1 < msg->range_min || r1 > msg->range_max) continue;

      const float a0 = pointAngFromRobot(static_cast<float>(j),     angle_inc, angle_min);
      const float a1 = pointAngFromRobot(static_cast<float>(j + 1), angle_inc, angle_min);
      const float diff = getDiffAngle(a0, r0, a1, r1); // rad

      if (std::fabs(diff) < 0.1f) { // ~5.7° de tolerancia de rectitud
        int   clusterSize = 2;
        int   goodCount   = 2;
        float angleCalc   = diff;

        while (std::fabs(angleCalc) < 0.1f) {
          const int idx = j + clusterSize;
          if (idx >= static_cast<int>(msg->ranges.size())) break;
          const float rnext = msg->ranges[idx];
          if (!std::isfinite(rnext) || rnext < msg->range_min || rnext > msg->range_max) {
            ++clusterSize;
            continue;
          }
          const float anext = pointAngFromRobot(static_cast<float>(idx), angle_inc, angle_min);
          angleCalc = getDiffAngle(a0, r0, anext, rnext);
          if (std::fabs(angleCalc) < 0.1f) {
            ++clusterSize;
            ++goodCount;
          } else break;
        }

        if (goodCount >= 2) {
          const int   endIdx = j + clusterSize - 1;
          const float aEnd   = pointAngFromRobot(static_cast<float>(endIdx), angle_inc, angle_min);
          const float rEnd   = msg->ranges[endIdx];

          const float clusterAngle = getDiffAngle(a0, r0, aEnd, rEnd);
          const float clusterMag   = getEuclideanDistance(a0, r0, aEnd, rEnd);

          clusters_.push_back(ClusterProps{
            clusterAngle,
            clusterMag,
            static_cast<float>(clusterSize),
            j
          });
        }

        j += clusterSize - 2; // salta para no recontar
      }
    }

    // (C) Publicar MarkerArray para Foxglove: una línea por clúster
    publish_markers(*msg);
    // (D) Publicar ángulo global para Foxglove
    if (std::isfinite(angle_deg)) {
      std_msgs::msg::Float32 msg_out;
      msg_out.data = angle_deg;
      angle_pub_->publish(msg_out);
    }
  }

  void publish_markers(const sensor_msgs::msg::LaserScan& scan) {
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;
    using geometry_msgs::msg::Point;

    MarkerArray arr;

    // Limpia todo lo anterior
    {
      Marker m;
      m.header = scan.header;
      m.action = Marker::DELETEALL;
      arr.markers.push_back(m);
    }

    for (std::size_t i = 0; i < clusters_.size(); ++i) {
      const auto& c = clusters_[i];

      const int j      = c.initPoint;
      const int endIdx = j + static_cast<int>(c.size) - 1;
      if (j < 0 || endIdx < 0 || endIdx >= static_cast<int>(scan.ranges.size())) continue;

      const float r0 = scan.ranges[j];
      const float r1 = scan.ranges[endIdx];
      if (!std::isfinite(r0) || !std::isfinite(r1)) continue;

      const float a0 = scan.angle_min + scan.angle_increment * static_cast<float>(j);
      const float a1 = scan.angle_min + scan.angle_increment * static_cast<float>(endIdx);

      geometry_msgs::msg::Point p0, p1;
      p0.x = r0 * std::cos(a0); p0.y = r0 * std::sin(a0); p0.z = 0.0;
      p1.x = r1 * std::cos(a1); p1.y = r1 * std::sin(a1); p1.z = 0.0;

      Marker line;
      line.header = scan.header;            // mismo frame del LIDAR
      line.ns     = "clusters";
      line.id     = static_cast<int>(i);    // id único por clúster
      line.type   = Marker::LINE_STRIP;
      line.action = Marker::ADD;

      line.scale.x = 0.03;                  // grosor en metros
      line.color.r = 0.1f; line.color.g = 0.8f; line.color.b = 0.2f; line.color.a = 1.0f;
      line.pose.orientation.w = 1.0;        // identidad
      line.points.push_back(p0);
      line.points.push_back(p1);

      arr.markers.push_back(line);
    }

    cluster_pub_->publish(arr);
  }

  void on_timer() {
    float deg = angle_deg_.load();
    uint16_t ang_tenths = 0;

    if (std::isfinite(deg)) {
      while (deg >= 360.0f) deg -= 360.0f;
      while (deg < 0.0f)    deg += 360.0f;
      ang_tenths = static_cast<uint16_t>(deg * 10.0f + 0.5f);
    } else {
      ang_tenths = 300; // 30.0° como valor por defecto
    }

    auto clamp_byte = [](float v)->uint8_t {
      if (v < 0.f)   v = 0.f;
      if (v > 255.f) v = 255.f;
      return static_cast<uint8_t>(v + 0.5f);
    };
    const uint8_t send_pwm = clamp_byte(pwm_.load());
    const uint8_t send_kp  = clamp_byte(kp_.load());

    auto frame = empaquetar(ang_tenths, send_pwm, send_kp);
    (void)serial_.write_bytes(frame.data(), frame.size());
  }

  // Serial
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr                                    timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            angle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr    control_sub_;

  // Estado
  std::atomic<float> angle_deg_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> pwm_{70.0f};
  std::atomic<float> kp_{2.0f};
  std::vector<ClusterProps> clusters_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}
