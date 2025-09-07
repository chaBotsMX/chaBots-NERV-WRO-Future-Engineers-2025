#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

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

    // ---- empaquetado ----
  static std::array<uint8_t, 6> pack(uint16_t err_deg, uint8_t pwm_byte, uint8_t dir) {
      std::array<uint8_t, 6> f{};
      f[0] = 0xAB;
      f[1] = static_cast<uint8_t>((err_deg >> 8) & 0xFF);
      f[2] = static_cast<uint8_t>(err_deg & 0xFF);
      f[3] = pwm_byte;    // velocidad
      f[4] = dir;
      uint8_t chk = 0; for (int i=0;i<5;++i) chk ^= f[i]; f[5] = chk;
      return f;
  }

  bool outOfParkingLot(){
    const float leftDistance = dist_Left_.load();
    const float rightDistance = dist_Right_.load();
    bool reversed = backInParking_.load();
    float Ypos = poseY_.load();
    int direction = direction_.load();
    if(!std::isfinite(leftDistance) && !std::isfinite(rightDistance)) return false;
    if(direction_.load() == 0){
      if(rightDistance > 0.5f) direction_.store(2);
      else if(leftDistance > 0.5f) direction_.store(1);
      return false;
    }
    direction = direction_.load();
    if(!frontInParking_.load()){
      if(Ypos < 0.03f){
        auto frame = pack(90, 30, 0);
        (void)serial_.write_bytes(frame.data(), frame.size());
        return false;
      }
      else{frontInParking_.store(true);}
    }
    else if(!changueDelay_.load()){
      if(direction == 1){
        auto frame = pack(150, 0, 0);
        (void)serial_.write_bytes(frame.data(), frame.size());
      }
      else if(direction == 2){
        auto frame = pack(50, 0, 0);
        (void)serial_.write_bytes(frame.data(), frame.size());
      }
      std::this_thread::sleep_for(1s);
      changueDelay_.store(true);
      return false;
    }
    else if(reversed == false){
      if(direction == 2 && Ypos > -0.05f){
        auto frame = pack(50, 45, 1);
        (void)serial_.write_bytes(frame.data(), frame.size());
        return false;
      }
      else if(direction == 1 && Ypos > -0.04f){
        auto frame = pack(150, 45, 1);
        (void)serial_.write_bytes(frame.data(), frame.size());
        return false;
      }
      else{backInParking_.store(true); return false;}
    }
    else if(!repeat_.load()){
        backInParking_.store(false);
        frontInParking_.store(false);
        changueDelay_.store(false);
        repeat_.store(true);

    }
    else if(!out_.load()){
      if(direction ==  1 && Ypos < 0.15f){
        auto frame = pack(50, 40, 0);
        (void)serial_.write_bytes(frame.data(), frame.size());
        return false;
      }
      else if(direction ==  2 && Ypos < 0.15f){
        auto frame = pack(150, 40, 0);
        (void)serial_.write_bytes(frame.data(), frame.size());
        return false;
      }
      else
      {
        out_.store(true);
        return false;
      }
    }
    else if(!initCorrection_.load()){
      if(direction == 1){
        auto frame = pack(150, 0, 0);
        (void)serial_.write_bytes(frame.data(), frame.size());
      }
      else if(direction == 2){
        auto frame = pack(50, 0, 0);
        (void)serial_.write_bytes(frame.data(), frame.size());
      }
      std::this_thread::sleep_for(1s);
      initCorrection_.store(true);
      return true;
    }
    else{
      return false;
    }
  }





  static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
  }

  const float kPI = 3.14159265358979323846f;

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

    if (error < -0.5f || targetSpeed == 0.0f) return 0; 
    if (error < -0.1f) return 1;                        
    return static_cast<int>(pwm);
  }

  // ---- callbacks ----
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const float a_min = msg->angle_min;
    const float a_inc = msg->angle_increment;

    float sumFront = 0.0f; size_t totalFront = 0;
    float sumLeft = 0.0f; size_t totalLeft = 0;
    float sumRight = 0.0f; size_t totalRight = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float a = a_min + a_inc * static_cast<float>(i);
      if (a < 0.0f || a > 3.141592f) continue; // ~70°–110°
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      if (a > 1.39 && a < 1.7453f) { sumFront += r; ++totalFront; }
      if( a > 0.0f && a < 0.5235f) { sumLeft += r; ++totalLeft; }
      if( a > 2.79252f && a < 3.141592f) { sumRight += r; ++totalRight; }
    }

    const float frontmean = (totalFront ? (sumFront / static_cast<float>(totalFront)) : std::numeric_limits<float>::quiet_NaN());
    dist_front_.store(frontmean);
    const float leftmean = (totalLeft ? (sumLeft / static_cast<float>(totalLeft)) : std::numeric_limits<float>::quiet_NaN());
    dist_Left_.store(leftmean);
    const float rightmean = (totalRight ? (sumRight / static_cast<float>(totalRight)) : std::numeric_limits<float>::quiet_NaN());
    dist_Right_.store(rightmean);
  }


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


  heading_acc_.store(acc);
  heading360_.store(wrap_360(acc));

  
  if (!std::isnan(msg->twist.twist.linear.z)) {
    speed_.store(msg->twist.twist.linear.z);
  }
    poseY_.store(msg->pose.pose.position.y);
}

// --- on_timer: guarda inicial y usa object_distance_ cuando haya objeto ---
  void on_timer() {
    // Guardias simples (evita bloquear por laterales NaN)
    if (!std::isfinite(heading360_.load())) return;

    // Constantes
    constexpr float EVADE_OFFSET_DEG = 30.0f;   // evasión máx
    constexpr float EVADE_MIN_DIST   = 30.0f;   // cm
    constexpr float EVADE_MAX_DIST   = 100.0f;  // cm
    constexpr float MAX_MAG          = 90.0f;
    constexpr float LARGE_ERR_DEG    = 5.0f;

    // Histeresis por ángulo de imagen (px de tu /object/angle)
    constexpr float ANGLE_INNER = 15.0f;  // activa evasión
    constexpr float ANGLE_OUTER = 25.0f;  // desactiva evasión
    outOfParking_.store(outOfParkingLot());
    bool initDrive = outOfParking_.load();

    if (initDrive == true) {
      const bool  hayObjeto = (object_status_.load() > 0.5f);
      const float d_front   = dist_front_.load();
      const bool  blocked   = (!hayObjeto) && std::isfinite(d_front) && (d_front < 1.0f);

      auto cycle_target = [&]() -> float {
        switch (cycle_idx_.load()) {
          case 0: return 0.0f;
          case 1: return 90.0f;
          case 2: return 180.0f;
          default: return 270.0f;
        }
      };

      // --- Estado de evasión con histeresis por /object/angle ---
      static bool evasive_active = false;
      const float objAngle = object_angle_.load();         // +derecha / -izquierda
      float Object_dist = object_distance_.load() / 2.0f;  // cm reales (tú envías cm)
      if (!std::isfinite(Object_dist)) Object_dist = EVADE_MAX_DIST;
      Object_dist = clampf(Object_dist, EVADE_MIN_DIST, EVADE_MAX_DIST);

      const bool in_corridor  = std::isfinite(objAngle) && (std::fabs(objAngle) <= ANGLE_INNER);
      const bool out_corridor = (!std::isfinite(objAngle)) || (std::fabs(objAngle) >= ANGLE_OUTER);

      if (hayObjeto && in_corridor)  evasive_active = true;
      if (!hayObjeto || out_corridor) evasive_active = false;

      float target_abs_deg = cycle_target();  // por defecto: setpoints
      const float h = heading360_.load();

      if (evasive_active) {
        const int color = static_cast<int>(object_color_.load()); // 0=VERDE, 1=ROJO
        const float alpha = (EVADE_MAX_DIST - Object_dist) / (EVADE_MAX_DIST - EVADE_MIN_DIST);
        const float evade = EVADE_OFFSET_DEG * alpha; // 0..30°

        if (color == 1) {           // ROJO → derecha
          target_abs_deg = h - evade;
        } else if (color == 0) {    // VERDE → izquierda
          target_abs_deg = h + evade;
        } else {
          target_abs_deg = cycle_target(); // color desconocido → setpoint
          RCLCPP_INFO(get_logger(), "[OBJ=SI] color_desconocido salida=%.1f°", target_abs_deg);
        }

        if (target_abs_deg < 0.0f)    target_abs_deg += 360.0f;
        if (target_abs_deg >= 360.0f) target_abs_deg -= 360.0f;

        RCLCPP_INFO(get_logger(),
          "[OBJ=SI] angle=%.1fpx dist=%.1fcm evade=%.1f° salida=%.1f° (EVADE=ON)",
          objAngle, Object_dist, evade, target_abs_deg);
      } else {
        // Sin evasión activa: sigue setpoints
        RCLCPP_INFO(get_logger(),
          hayObjeto ? "[OBJ=SI] fuera de trayectoria (EVADE=OFF)"
                    : "[OBJ=NO] (EVADE=OFF)");
      }

      // --- delta firmado en [-180,180)
      const float heading = heading360_.load();
      float delta = target_abs_deg - heading;
      delta = std::fmod(delta + 180.0f, 360.0f);
      if (delta < 0.0f) delta += 360.0f;
      delta -= 180.0f;

      // Tu lógica de cambio de setpoint por LiDAR
      const bool consider_block = (!hayObjeto) && (std::fabs(delta) <= LARGE_ERR_DEG);
      const bool last_blk = last_blocked_.load();
      if (consider_block && blocked && !last_blk) {
        int idx = cycle_idx_.load();
        idx = (idx + 1) & 3;
        cycle_idx_.store(idx);
      }
      last_blocked_.store(consider_block && blocked);

      float mag = std::min(std::fabs(delta), MAX_MAG);
      float input_deg = 90.0f + ((delta >= 0.0f) ? +mag : -mag);
      input_deg = clampf(input_deg, 0.0f, 180.0f);

      const uint16_t send_deg = static_cast<uint16_t>(std::lround(input_deg));
      const uint8_t  pwm = 50;
      const uint8_t  dir = 0; // <<< pack espera 'dir' como 3er byte
      auto frame = pack(send_deg, pwm, dir);
      (void)serial_.write_bytes(frame.data(), frame.size());
    }
    else {
      // NO toco tu feature de estacionamiento
      outOfParking_.store(outOfParkingLot());
    }
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
  std::atomic<bool> out_{false};
  std::atomic<bool> initCorrection_{false};
  std::atomic<bool> repeat_{false};
  std::atomic<bool> changueDelay_{false};
  std::atomic<bool> frontInParking_{false};
  std::atomic<bool> backInParking_{false};
  std::atomic<bool> outOfParking_{false};
  std::atomic<int> direction_{0}; // 1 = "LEFT", 2 = "RIGHT"
  std::atomic<float> dist_Left_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> dist_Right_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> poseY_{std::numeric_limits<float>::quiet_NaN()};
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
  std::this_thread::sleep_for(std::chrono::seconds(5));
  rclcpp::spin(std::make_shared<TeensyObsNode>());
  rclcpp::shutdown();
  return 0;
}
