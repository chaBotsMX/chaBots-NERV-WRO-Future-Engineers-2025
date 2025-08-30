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
  // Para PID: guardar el error anterior
  float prev_error_ = 0.0f;
  std::chrono::steady_clock::time_point prev_time_ = std::chrono::steady_clock::now();
public:
  TeensyCommNode() : Node("teensy_comm") {
    (void)serial_.open_port();

    angle_pub_   = create_publisher<std_msgs::msg::Float32>("/send_angle", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),std::bind(&TeensyCommNode::on_scan, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10,std::bind(&TeensyCommNode::on_odom, this, std::placeholders::_1));
    timer_ = create_wall_timer(10ms, std::bind(&TeensyCommNode::on_timer, this));
  }

private:
  // --------- Utilidades trig/cartesianas ----------
  static inline float pointAngFromRobot(float iteration, float angleInc, float minAngle){
    return  minAngle + angleInc * iteration; // rad
  }
  static inline float deg2rad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }
  static inline float rad2deg(float rad) { return rad * 180.0f / static_cast<float>(M_PI); }
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
  
  void getOffsetsFromLidar( const sensor_msgs::msg::LaserScan::SharedPtr msg){

    float rightDis = 0;
    float leftDis = 0;
    float frontDis = 0;
    int sum_left = 0;
    int sum_right = 0;
    int sum_front = 0;  
    float totalDis = 0;
    float setpoint = 0;
    for(int i = 0; i < msg->ranges.size(); ++i) {
      const float ang = msg->angle_min + msg->angle_increment * static_cast<float>(i);
      if (ang < 0.0f || ang > M_PI) continue; // 0..180°
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
        if (ang < 0.78f){
          leftDis += r * std::cos(ang+(deg2rad(headingSetPoint.load()) - deg2rad(heading360.load())));
          sum_left++;
        }     
        if(ang >  2.35f){
          rightDis += r * std::cos(ang-(deg2rad(headingSetPoint.load()) - deg2rad(heading360.load())));
          sum_right++;
        }
        if(ang >= 1.39f && ang <= 1.74f){
          frontDis += r * std::sin(ang+(deg2rad(headingSetPoint.load()) - deg2rad(heading360.load())));
          sum_front++;
        }
      }
      leftDis /= sum_left;
      rightDis /= sum_right;
      frontDis /= sum_front;
      totalDis = std::fabs(leftDis) +  std::fabs(rightDis);
      setpoint = totalDis / 2.0f;
      frontWallDistance.store(frontDis);
      centeringOffset.store(setpoint - std::fabs(rightDis));
  }

  float headingError(const sensor_msgs::msg::LaserScan::SharedPtr msg, float kp){
    float heading = heading360.load();
    float error = headingSetPoint.load() - heading;
    return error;
  }



// -----------maquina de estados ----------

  void state_machine() {
    if (laps.load() == 1) {
      getMiddleAndEnd();
    } else if (end.load() == 1) {
      getEnd();
    } else if (safe.load() == 1) {
      goAngleAndCenter();
    } else if (turn.load() == 1) {
      turnRobot();
    } else if (dis.load() == 0) {
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
   /* float vecCenter = getOffsetFromCenter(msg, kp_);  
    float vecHeading = headingError(msg, kp_);
    float vecFinal = vecCenter + vecHeading;
    */
  }

  void turnRobot() {
    // Implementar lógica para girar el robot
  }

  void searchHole() {
    // Implementar lógica para buscar un agujero
  }


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
  float clampf(float v, float lo, float hi) {
      return std::max(lo, std::min(v, hi));
  }

    float feedforward_pwm_multiplier(float targetSpeed) {
    // Si la velocidad es menor o igual a 0, no hay PWM
    if (targetSpeed <= 0.0f) return 0.0f;
    // Puntos de referencia
    const float v1 = 0.5f, pwm1 = 25.0f;
    const float v2 = 2.0f, pwm2 = 60.0f;
    if (targetSpeed <= v1) return pwm1;
    if (targetSpeed >= v2) return pwm2;
    // Interpolación lineal entre los dos puntos
    float pwm = pwm1 + (pwm2 - pwm1) * (targetSpeed - v1) / (v2 - v1);
    return pwm;
  }

  int controlACDA(float targetSpeed){
  float pwm = 0, jerk = 10;
  float error = targetSpeed - speed.load();
  float aproxPwm = feedforward_pwm_multiplier(targetSpeed); // PWM feedforward no lineal
  float lastPwmLocal = lastPwm.load();
  float kp = 8.25f; // Valor a determinar
  float kd = 0.1f; // Valor a determinar
  pwm = (error * kp)  + ((error - lastError.load()) / 0.01) * kd;
  pwm = clampf(clampf(pwm + aproxPwm, lastPwmLocal - jerk, lastPwmLocal + jerk),0,255);
  lastPwm.store(pwm);
  lastError.store(error);
  if(error < -0.5f || targetSpeed == 0) return 0;
  if (error < -0.1f) return 1;
  return static_cast<int>(pwm);
  }

  //callbacks que se ejecutan al leer un topic
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const float angle_min = msg->angle_min;
    const float angle_inc = msg->angle_increment;
    const float pi        = static_cast<float>(M_PI);
    double sum_x = 0.0, sum_y = 0.0;
    size_t used = 0;
  
    getOffsetsFromLidar(msg);
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
    absolute_angle.store(angle_deg);
  }

  void on_timer() {

  float front = frontWallDistance.load();
  float offset = centeringOffset.load();
  float deg = angle_deg_.load();
  float degrees = heading360.load();
  float head = heading.load();
  float current_speed = speed.load();
  int returnPWM = 0;

  if(frontWallDistance < 0.8f){returnPWM = controlACDA(0.5f);}
  else if(frontWallDistance > 1.5f){
    returnPWM = controlACDA(1.4f);
  }
  else{
    returnPWM = controlACDA(0.8f);
  }

  float heading = heading360.load();
  deg = std::fmod((0.0f - heading + 540.0f), 360.0f) - 180.0f; 
  RCLCPP_INFO(this->get_logger(), "distancia al frente: %f, offset: %f, angulo: %f, correcion IMU: %f, velocidad: %f, vel_cmd: %d", front, offset, degrees, deg, current_speed, returnPWM);
  auto frame = empaquetar(static_cast<uint16_t>(absolute_angle.load()), returnPWM, 10,this->get_logger());
  (void)serial_.write_bytes(frame.data(), frame.size());
  }


  static inline float wrap_360(float a) { float x = std::fmod(a, 360.0f); return (x < 0) ? x + 360.0f : x; }

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
    heading360.store(wrap_360(acc));

    // Leer la velocidad absoluta publicada por otos_reader (en linear.z)
    if (msg->twist.twist.linear.z == msg->twist.twist.linear.z) { // check for NaN
      speed.store(msg->twist.twist.linear.z);
    }
  }
  // Serial
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr                                    timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            angle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // variables persistentes
  std::atomic<int> lastPwm{0};
  std::atomic<float> lastError{0.0f};
  std::atomic<float> centeringOffset{0.0f};
  std::atomic<float> frontWallDistance{0.0f};
  std::atomic<float> headingSetPoint{0.0f};
  std::atomic<float> heading360{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> angle_deg_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> absolute_angle{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> pwm_{70.0f};
  std::atomic<float> kp_{2.0f};
  std::atomic<float> heading{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<bool> dis{false};
  std::atomic<bool> turn{false};
  std::atomic<bool> safe{true};
  std::atomic<bool> end{false};
  std::atomic<bool> laps{false};
  std::atomic<float> speed{0.0f};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}
