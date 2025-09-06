#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <thread>
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
  float anchos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
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
    const float phi = (90.0f - absolute_angle.load()) * static_cast<float>(M_PI) / 180.0f;

    for (int i = 0; i < static_cast<int>(msg->ranges.size()); ++i) {
      const float ang = msg->angle_min + msg->angle_increment * static_cast<float>(i);
      if (ang < 0.0f || ang > static_cast<float>(M_PI)) continue;

      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;

      const float ang_eff = ang + phi;  // MISMA rotación para todos

      // Bandas en el frame rotado (0=izq, π/2=frente, π=der)
      if (ang_eff >= 0.0f && ang_eff < 0.78f) {                 // izquierda ~ 0..45°
        leftDis  += r * std::cos(ang_eff - 0.0f);
        ++sum_left;
      }
      if (ang_eff > 2.35f && ang_eff <= static_cast<float>(M_PI)) { // derecha ~ 135..180°
        rightDis += r * std::cos(ang_eff - static_cast<float>(M_PI));
        ++sum_right;
      }
      if (ang_eff >= 1.39f && ang_eff <= 1.74f) {               // frente ~ 80..100°
        frontDis += r;  // (o usa r * cos(ang_eff - M_PI_2) si quieres la normal frontal)
        ++sum_front;
      }
    }
      leftDis /= sum_left;
      rightDis /= sum_right;
      frontDis /= sum_front;
      totalDis = std::fabs(leftDis) +  std::fabs(rightDis);
      anchoCorredor.store(totalDis);
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

  void getOptimalValues() {
    if(driveDirection.load() == 1){ //horario
      if(anchos[0]){

      }
    } else if(driveDirection.load() == 2){//antihorario
      if(anchos[0]){

      }
    }
  } 
  void getSector(){
    float orientation = heading360.load();
    if(anchos[0] != 0 && anchos[1] != 0 && anchos[2] != 0 && anchos[3] != 0){
      firstLap.store(false);
    }
    if(orientation >= 315 || orientation < 45){
      if(firstLap.load()){
        anchos[0] = anchoCorredor.load();
      }
        actualSector.store(1);
    } else if(orientation >= 45 && orientation < 135){
      if(firstLap.load()){
        anchos[1] = anchoCorredor.load();
      }
        actualSector.store(2);
    } else if(orientation >= 135 && orientation < 225){
      if(firstLap.load()){
        anchos[2] = anchoCorredor.load();
      }
        actualSector.store(3);
    } else if(orientation >= 225 && orientation < 315){
        if(firstLap.load()){
          anchos[3] = anchoCorredor.load();
          }
        actualSector.store(4);
    }
  }

  static std::array<uint8_t, 6> empaquetar(uint16_t ang_tenths, uint8_t pwm_byte, uint8_t dir, rclcpp::Logger logger) {
    std::array<uint8_t, 6> f{};
    f[0] = 0xAB;
    f[1] = static_cast<uint8_t>((ang_tenths >> 8) & 0xFF);
    f[2] = static_cast<uint8_t>(ang_tenths & 0xFF);
    f[3] = pwm_byte;
    f[4] = dir;
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
    float feedpwm = pwm1 + (pwm2 - pwm1) * (targetSpeed - v1) / (v2 - v1);
    return feedpwm;
  }

  int controlACDA(float targetSpeed){
    float pwm = 0, jerk = 10;
    float error = targetSpeed - speed.load();
    float aproxPwm = 35.0f;
    if(targetSpeed < 0.6f){aproxPwm = 35.0f;}
    else if(targetSpeed < 1.2f){aproxPwm = 40.0f;}
    else{aproxPwm = 60.0f;}
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

  float angleProccesing(float kpNoLinear = 0.75f, float maxOut = 30.0f){
      float angleInput = absolute_angle.load();
      float angularError = 90.0f - angleInput;
      float beta = kpNoLinear/maxOut;
      return maxOut * std::tanh(angularError / (maxOut / kpNoLinear));
    }

float objectiveAngleVelPD(float vel_min, float vel_max){
  const float alpha = 0.3f;   // suavizado EMA
  const float dt    = 0.01f;  // 10 ms (tu timer)

  float a = absolute_angle.load();
  // Si quieres evitar spikes al primer ciclo sin dato:
  if (!std::isfinite(a)) return vel_min;   // sin reducción cuando no hay ángulo

  // Error envuelto a [-180, 180]
  float e = 90.0f - a;
  while (e > 180.0f)  e -= 360.0f;
  while (e < -180.0f) e += 360.0f;

  // Derivada cruda con el error previo
  float e_prev       = lastVelErr.load();
  float raw_derivada = (e - e_prev) / dt;      // deg/s "amplificado"
  lastVelErr.store(e);

  // EMA correcto: y(k) = y(k-1) + alpha * (x(k) - y(k-1))
  float der_prev  = de_f.load();
  float derivada  = der_prev + alpha * (raw_derivada - der_prev);
  de_f.store(derivada);

  // (Opcional) tope duro a la derivada filtrada para eliminar picos extremos
  // derivada = clampf(derivada, -400.0f, 400.0f);
  const float kp = 0.04f;  // m/s por grado
  const float kd = 0.005f; // m/s por (grado/seg filtrado)
  float reduccion = kp * std::fabs(e); /*+ kd * std::fabs(derivada);*/

  return clampf(reduccion, vel_min, vel_max);  // p.ej. [0.0f, 0.8f]
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
      if (ang < -0.5235f && ang > -2.6180f || ang > pi) continue; // 0..180°
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
    getOffsetsFromLidar(msg);
  }

  void on_timer() {

  float front = frontWallDistance.load();
  float offset = centeringOffset.load();
  float degrees = heading360.load();
  float head = heading.load();
  float current_speed = speed.load();
  int returnPWM = 0;
  int dir = 0;
  bool ending = endRound.load();
  float sendAngle = absolute_angle.load();

   if(firstLap.load()){
      getSector();
      if(front <= 0.4f){
        returnPWM = controlACDA(0.5f);
        sendAngle = 90 + -angleProccesing( 0.80f, 30.0f);
      }
      else if(front > 0.4f && front <= 1.4f){
        returnPWM = controlACDA(0.8f) - fabs(objectiveAngleVelPD(0.0f, 0.3f));
        sendAngle = 90 + -angleProccesing( 0.70f, 50.0f);
      }
      else if(front <= 1.4f){
        returnPWM = controlACDA(1.0f) - fabs(objectiveAngleVelPD(0.0f, 0.5f));
        sendAngle = 90 + -angleProccesing( 0.5f, 50.0f);
      }
      else if(front > 1.4f){
        returnPWM = controlACDA(1.8f - fabs(objectiveAngleVelPD(0.0f, 1.2f)));
        sendAngle = 90 + -angleProccesing( 0.50f, 60.0f);
      }
      else{
        returnPWM = controlACDA(1.0f - fabs(objectiveAngleVelPD(0.0f, 0.4f)));
        sendAngle = 90 + -angleProccesing(0.75f, 30.0f);
      }
      if(!init.load()){
        if(std::isfinite(absolute_angle.load()) && front != 0.0f){
      
          init.store(true);
        }
        returnPWM = 0;
      }
    RCLCPP_INFO(this->get_logger(), "distancia al frente: %f, offset: %f, angulo: %f, correcion IMU: %f, velocidad: %f, vel_cmd: %d", front, offset, degrees, head, current_speed, returnPWM);
    auto frame = empaquetar(static_cast<uint16_t>(sendAngle), returnPWM, dir,this->get_logger());
    (void)serial_.write_bytes(frame.data(), frame.size());

    }
  else if(ending == false){
    if(front <= 0.4f){
      returnPWM = controlACDA(0.5f);
      sendAngle = 90 + -angleProccesing( 0.80f, 30.0f);
    }
    else if(front > 0.4f && front <= 1.4f){
      returnPWM = controlACDA(0.8f) - fabs(objectiveAngleVelPD(0.0f, 0.3f));
      sendAngle = 90 + -angleProccesing( 0.70f, 50.0f);
    }
    else if(front <= 1.4f){
      returnPWM = controlACDA(1.0f) - fabs(objectiveAngleVelPD(0.0f, 0.5f));
      sendAngle = 90 + -angleProccesing( 0.5f, 50.0f);
    }
    else if(front > 1.4f){
      returnPWM = controlACDA(1.8f - fabs(objectiveAngleVelPD(0.0f, 1.2f)));
      sendAngle = 90 + -angleProccesing( 0.50f, 60.0f);
    }
    else{
      returnPWM = controlACDA(1.0f - fabs(objectiveAngleVelPD(0.0f, 0.4f)));
      sendAngle = 90 + -angleProccesing(0.75f, 30.0f);
    }

    float heading = heading360.load();
    if (std::fabs(head) > 1076.0f && front < 1.8f) { // check for NaN
      endRound.store(true);
      returnPWM = 0;
    }
    RCLCPP_INFO(this->get_logger(), "distancia al frente: %f, offset: %f, angulo: %f, correcion IMU: %f, velocidad: %f, vel_cmd: %d", front, offset, degrees, head, current_speed, returnPWM);
    auto frame = empaquetar(static_cast<uint16_t>(sendAngle), returnPWM, dir,this->get_logger());
    (void)serial_.write_bytes(frame.data(), frame.size());
  }
  else{
    if(front < 1.2){
      auto frame = empaquetar(static_cast<uint16_t>(90), 50, 1,this->get_logger());
      (void)serial_.write_bytes(frame.data(), frame.size());
    }
    else if(front > 1.8){
      auto frame = empaquetar(static_cast<uint16_t>(90), 50, 0,this->get_logger());
      (void)serial_.write_bytes(frame.data(), frame.size());
    }
    else{
      auto frame = empaquetar(static_cast<uint16_t>(90), 0, 1,this->get_logger());
      (void)serial_.write_bytes(frame.data(), frame.size());
    }
  }
  }

  static inline float wrap_360(float a) { float x = std::fmod(a, 360.0f); return (x < 0) ? x + 360.0f : x; }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {

  const auto& q = msg->pose.pose.orientation;
  float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  float yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0f / static_cast<float>(M_PI);

  static bool init = false;
  static float prev = 0.0f;
  static float acc = 0.0f;
  if (!init) {
    prev = yaw_deg;
    acc = 0.0f;
    init = true;
  } else {
    float d = yaw_deg - prev;

    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    acc += d;
    prev = yaw_deg;
  }
  heading.store(acc);
  heading360.store(wrap_360(std::fmod(acc, 360.0f)));

    
    if (msg->twist.twist.linear.z == msg->twist.twist.linear.z) { 
      speed.store(msg->twist.twist.linear.z);
    }
  }
  
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr                                    timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            angle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // variables persistentes
  std::atomic<int> actualSector{0};
  std::atomic<int> driveDirection{0}; //0 no determinado, 1 sentido horario, 2 sentido antihorario
  std::atomic<bool>firstLap{true};
  std::atomic<float> anchoCorredor{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<bool> endRound{false};
  std::atomic<bool> init{false};
  std::atomic<int> lastPwm{0};
  std::atomic<float> de_f{0.0f};
  std::atomic<float> lastVelErr{0.0f};
  std::atomic<float> lastError{0.0f};
  std::atomic<float> centeringOffset{0.0f};
  std::atomic<float> frontWallDistance{0.0f};
  std::atomic<float> headingSetPoint{0.0f};
  std::atomic<float> heading360{std::numeric_limits<float>::quiet_NaN()};
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
  std::this_thread::sleep_for(std::chrono::seconds(3));
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}
