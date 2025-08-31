#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

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

class SerialPort {
public:
  SerialPort() : fd_(-1) {}
  ~SerialPort() { close_port(); }

  bool open_port(const char* dev = "/dev/ttyAMA0", speed_t baud=B2000000) {
    close_port();
    fd_ = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) { last_error_ = std::string("open: ") + std::strerror(errno); return false; }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) { last_error_ = std::string("tcgetattr: ")+std::strerror(errno); close_port(); return false; }
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB; // 8N1
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) { last_error_ = std::string("tcsetattr: ")+std::strerror(errno); close_port(); return false; }
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close_port() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

  bool write_bytes(const uint8_t* data, size_t len) {
    if (fd_ < 0) return false;
    size_t sent = 0;
    while (sent < len) {
      ssize_t n = ::write(fd_, data + sent, len - sent);
      if (n < 0) { if (errno == EAGAIN || errno == EWOULDBLOCK) continue; return false; }
      sent += static_cast<size_t>(n);
    }
    return true;
  }

  std::string last_error() const { return last_error_; }

private:
  int fd_;
  std::string last_error_;
};

static inline float wrap_360(float a) { float x = std::fmod(a, 360.0f); return (x < 0) ? x + 360.0f : x; }
static inline float wrap_pm180(float a) { float x = std::fmod(a + 180.0f, 360.0f); if (x < 0) x += 360.0f; return x - 180.0f; }

class TeensyObsNode : public rclcpp::Node {
public:
  TeensyObsNode() : Node("teensy_obs") {
    (void)serial_.open_port();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&TeensyObsNode::on_scan, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&TeensyObsNode::on_odom, this, std::placeholders::_1));

    // Nodo de visión
    object_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/object/distance", 10, [this](std_msgs::msg::Float32::SharedPtr m){ object_distance_.store(m->data); });
    object_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/object/angle", 10, [this](std_msgs::msg::Float32::SharedPtr m){ object_angle_.store(m->data); });
    object_color_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/object/color", 10, [this](std_msgs::msg::Float32::SharedPtr m){ object_color_.store(m->data); });
    object_status_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/object/status", 10, [this](std_msgs::msg::Float32::SharedPtr m){ object_status_.store(m->data); });

    // Estado inicial
    target_deg_ = 0.0f;            // 0°
    cycle_idx_.store(0);
    last_blocked_.store(false);

  timer_ = create_wall_timer(10ms, std::bind(&TeensyObsNode::on_timer, this));
  }

private:
  // ---- callbacks ----
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const float a_min = msg->angle_min;
    const float a_inc = msg->angle_increment;

    float sum = 0.0f; size_t n = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float a = a_min + a_inc * static_cast<float>(i);
      if (a < 1.22f || a > 1.91f) continue; // ~70°–110°
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      sum += r; ++n;
    }
    const float mean = (n ? (sum / static_cast<float>(n)) : std::numeric_limits<float>::quiet_NaN());
    dist_front_.store(mean);
  }

  const float kPI = 3.14159265358979323846f;
// --- on_odom: corrige nombres y M_PI ---
void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto& q = msg->pose.pose.orientation;
  float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  float yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0f / kPI;

  static bool init = false;
  static float prev = 0.0f;
  static float acc = 0.0f;
  if (!init) {
    prev = yaw_deg;
    acc = 0.0f;
    init = true;
  } else {
    float d = yaw_deg - prev;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    acc += d;
    prev = yaw_deg;
  }

  // FIX: nombres correctos
  heading_acc_.store(acc);
  heading360_.store(wrap_360(acc));

  // velocidad absoluta (si no es NaN)
  if (!std::isnan(msg->twist.twist.linear.z)) {
    speed_.store(msg->twist.twist.linear.z);
  }
}
  // ---- empaquetado ----
  static std::array<uint8_t, 6> pack(uint16_t err_deg, uint8_t pwm_byte, uint8_t kp) {
    std::array<uint8_t, 6> f{};
    f[0] = 0xAB;
    f[1] = static_cast<uint8_t>((err_deg >> 8) & 0xFF);
    f[2] = static_cast<uint8_t>(err_deg & 0xFF);
    f[3] = pwm_byte;    // velocidad
    f[4] = kp;   
    uint8_t chk = 0; for (int i=0;i<5;++i) chk ^= f[i]; f[5] = chk;
    return f;
  }

static inline float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(v, hi));
}


// --- controlACDA: usa miembros y clampf ---
int controlACDA(float targetSpeed){
  float pwm = 0, jerk = 10;
  float error = targetSpeed - speed_.load();
  float aproxPwm;
  if      (targetSpeed < 0.6f) aproxPwm = 35.0f;
  else if (targetSpeed < 1.2f) aproxPwm = 40.0f;
  else                         aproxPwm = 60.0f;

  float lastPwmLocal = lastPwm_.load();
  float kp = 8.25f;
  float kd = 0.1f;
  pwm = (error * kp) + ((error - lastError_.load()) / 0.01f) * kd;
  pwm = clampf(pwm + aproxPwm, lastPwmLocal - jerk, lastPwmLocal + jerk);
  pwm = clampf(pwm, 0.0f, 255.0f);
  lastPwm_.store(pwm);
  lastError_.store(error);

  if (error < -0.5f || targetSpeed == 0.0f) return 0; // brake
  if (error < -0.1f) return 1;                        // coast
  return static_cast<int>(pwm);
}



// --- on_timer: guarda inicial y usa object_distance_ cuando haya objeto ---
void on_timer() {
  if (!std::isfinite(heading360_.load())) return; // aún no hay odom

  // Constantes originales + bang-bang
  constexpr float EVADE_OFFSET_DEG = 30.0f;   // bang-bang: ±30° respecto a 90
  constexpr float EVADE_MIN_DIST   = 0.25f;   // (se mantiene por si usas después)
  constexpr float EVADE_MAX_DIST   = 1.75f;   // (se mantiene por si usas después)
  constexpr float MAX_MAG          = 90.0f;
  constexpr float LARGE_ERR_DEG    = 5.0f;

  const bool  has_object = (object_status_.load() == 1.0f);
  const float d_front    = dist_front_.load();
  const bool  blocked    = (!has_object) && std::isfinite(d_front) && (d_front < 1.0f);

  // --- BANG-BANG directo cuando hay objeto ---
  if (has_object) {
    const int color = static_cast<int>(object_color_.load()); // 0=verde, 1=rojo

    if (color == 1) {
      // ROJO -> derecha: 90 + 30
      const float input_deg = clampf(90.0f + EVADE_OFFSET_DEG, 0.0f, 180.0f);
      RCLCPP_INFO(get_logger(), "[OBJ=SI] color=ROJO salida=%.1f°", input_deg);

      const uint16_t send_deg = static_cast<uint16_t>(std::lround(input_deg));
      const uint8_t  pwm = 50;
      const uint8_t  kp  = 10;
      auto frame = pack(send_deg, pwm, kp);
      (void)serial_.write_bytes(frame.data(), frame.size());
      return; // evitamos la rama de setpoints/heading
    }
    if (color == 0) {
      // VERDE -> izquierda: 90 - 30
      const float input_deg = clampf(90.0f - EVADE_OFFSET_DEG, 0.0f, 180.0f);
      RCLCPP_INFO(get_logger(), "[OBJ=SI] color=VERDE salida=%.1f°", input_deg);

      const uint16_t send_deg = static_cast<uint16_t>(std::lround(input_deg));
      const uint8_t  pwm = 50;
      const uint8_t  kp  = 10;
      auto frame = pack(send_deg, pwm, kp);
      (void)serial_.write_bytes(frame.data(), frame.size());
      return; // evitamos la rama de setpoints/heading
    }

    // color desconocido -> seguimos flujo normal pero imprimimos que sí ve
    RCLCPP_INFO(get_logger(), "[OBJ=SI] color_desconocido");
  } else {
    // NO ve objeto
    RCLCPP_INFO(get_logger(), "[OBJ=NO]");
  }

  // --- SIN objeto (o color desconocido): tu lógica original de setpoints/heading ---
  float target_abs_deg;
  if (has_object) {
    // color desconocido => sigue ciclo
    switch (cycle_idx_.load()) {
      case 0: target_abs_deg = 0.0f;   break;
      case 1: target_abs_deg = 90.0f;  break;
      case 2: target_abs_deg = 180.0f; break;
      default: target_abs_deg = 270.0f; break;
    }
  } else {
    switch (cycle_idx_.load()) {
      case 0: target_abs_deg = 0.0f;   break;
      case 1: target_abs_deg = 90.0f;  break;
      case 2: target_abs_deg = 180.0f; break;
      default: target_abs_deg = 270.0f; break;
    }
  }

  // --- delta firmado en [-180,180)
  const float heading = heading360_.load();
  float delta = target_abs_deg - heading;
  delta = std::fmod(delta + 180.0f, 360.0f);
  if (delta < 0.0f) delta += 360.0f;
  delta -= 180.0f;

  const bool consider_block = (!has_object) && (std::fabs(delta) <= LARGE_ERR_DEG);

  const bool last_blk = last_blocked_.load();
  if (consider_block && blocked && !last_blk) {
    int idx = cycle_idx_.load();
    idx = (idx + 1) & 3;          // 0->1->2->3->0
    cycle_idx_.store(idx);
  }
  last_blocked_.store(consider_block && blocked);

  float mag = std::min(std::fabs(delta), MAX_MAG);
  float input_deg = 90.0f + ((delta >= 0.0f) ? +mag : -mag);
  input_deg = clampf(input_deg, 0.0f, 180.0f);

  const uint16_t send_deg = static_cast<uint16_t>(std::lround(input_deg));
  const uint8_t  pwm = 50;  // placeholders
  const uint8_t  kp  = 10;  // placeholders

  auto frame = pack(send_deg, pwm, kp);
  (void)serial_.write_bytes(frame.data(), frame.size());
}

// ---- miembros ----
  SerialPort serial_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr object_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr object_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr object_color_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr object_status_sub_;

  // atómicos (siempre .load() / .store())
  std::atomic<float> speed_{0.0f};
  std::atomic<float> lastPwm_{0.0f};
  std::atomic<float> lastError_{0.0f};
  std::atomic<float> heading_acc_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> heading360_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> dist_front_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> object_distance_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> object_angle_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> object_color_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> object_status_{0.0f};
  std::atomic<int>   cycle_idx_{0};
  std::atomic<bool>  last_blocked_{false};

  // locales al hilo del timer
  float target_deg_{0.0f};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyObsNode>());
  rclcpp::shutdown();
  return 0;
}
