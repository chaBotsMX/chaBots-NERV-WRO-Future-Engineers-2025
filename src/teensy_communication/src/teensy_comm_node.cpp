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


struct lidarPoints {
  float angle;  // rad o deg, tú decides
  float x;
  float y;
  float mag;
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
  // Para PID: guardar el error anterior
  float prev_error_ = 0.0f;
  std::chrono::steady_clock::time_point prev_time_ = std::chrono::steady_clock::now();
public:
  TeensyCommNode() : Node("teensy_comm") {
    (void)serial_.open_port();
    lidarMSG.reserve(5000);  
    angle_pub_   = create_publisher<std_msgs::msg::Float32>("/send_angle", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),std::bind(&TeensyCommNode::on_scan, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10,std::bind(&TeensyCommNode::on_odom, this, std::placeholders::_1));
    timer_ = create_wall_timer(10ms, std::bind(&TeensyCommNode::on_timer, this));
  }

private:
  std::vector<lidarPoints> lidarMSG;
  
  // --------- Utilidades trig/cartesianas ----------

inline float wrapPi(float a) {
  while (a <= -M_PI) a += 2.0f*M_PI;
  while (a >   M_PI) a -= 2.0f*M_PI;
  return a;
}
  static inline float pointAngFromRobot(float iteration, float angleInc, float minAngle){
    return  minAngle + angleInc * iteration; // rad
  }
  float sectores[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float sectoresAngs[2][4] = {{45.0f, 135.0f, 225.0f, 315.0f},
                              {315.0f, 45.0f, 135.0f, 225.0f}};

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
  

  void getOffsetsFromLidar(){
    float updateX = 0;
    float updateY = 0;
    float updateYaw = 0;
    if(new_otos_data.load()){
      updateX = posX_.load() - lastPosX.load();
      updateY = posY_.load() - lastPosY.load();
      updateYaw = wrapPi(deg2rad(yaw.load() - lastYaw.load()));
      for (auto& s : lidarMSG) {
        s.x += updateX;
        s.y += updateY;
        s.angle = wrapPi(atan2(s.y, s.x) - updateYaw);
        s.mag = std::hypot(s.x, s.y);
      }
      lastPosX.store(posX_.load());
      lastPosY.store(posY_.load());
      lastYaw.store(yaw.load());
      new_otos_data.store(false);
    } 

    float rightDis = 0;
    float leftDis = 0;
    float frontDis = 0;
    int sum_left = 0;
    int sum_right = 0;
    int sum_front = 0;  
    float totalDis = 0;
    float setpoint = 0;
    const float phi = (90.0f - absolute_angle.load()) * static_cast<float>(M_PI) / 180.0f;
    float sumX = 0, sumY = 0;
    for (const auto& s : lidarMSG) {
      const float ang = s.angle;
      const float ang_eff = ang + phi;  // MISMA rotación para todos
      sumX += s.x;
      sumY += s.y;

      if (ang_eff >= 0.0f && ang_eff < 0.78f) {                 // izquierda ~ 0..45°
        leftDis  += s.mag * std::cos(ang_eff - 0.0f);
        ++sum_left;
      }
      if (ang_eff > 2.35f && ang_eff <= static_cast<float>(M_PI)) { // derecha ~ 135..180°
        rightDis += s.mag * std::cos(ang_eff - static_cast<float>(M_PI));
        ++sum_right;
      }
      if (ang_eff >= 1.39f && ang_eff <= 1.74f) {               // frente ~ 80..100°
        frontDis += s.mag * std::cos(ang_eff - M_PI_2);
        ++sum_front;
      }
    }
      absolute_angle.store(rad2deg(std::atan2(sumY, sumX)));
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



  float getNextSectorSize(){
    int thisSector = actualSector.load();
    if(driveDirection.load() == 1){ //horario
      if(thisSector == 0){
        return sectores[1];
      } else if(thisSector == 1){
        return sectores[2];
      } else if(thisSector == 2){
        return sectores[3];
      } else if(thisSector == 3){
        return sectores[0];
      }
    } else if(driveDirection.load() == 2){//antihorario
      if(thisSector == 0){
        return sectores[3];
      } else if(thisSector == 1){
        return sectores[0];
      } else if(thisSector == 2){
        return sectores[1];
      } else if(thisSector == 3){
        return sectores[2];
      }
    }
  } 
  float getCurrentSectorSize(){
    int thisSector = actualSector.load();
    return sectores[thisSector];
  }   
  void getOptimalValues() {
    float actualSize = getCurrentSectorSize();
    float nextSize = getNextSectorSize();
    if(actualSize >= 0.80f){
      if(nextSize >= 0.80f){
        optimalSpeed.store(3.0f);
        optimalKp.store(0.40f);
        optimalSpeedTurn.store(1.5f);
        optimalKpTurn.store(0.40f);
      }
      else if(nextSize < 0.80f){
        optimalSpeed.store(1.0f);
        optimalKp.store(0.50f);
        optimalSpeedTurn.store(0.8f);
        optimalKpTurn.store(0.80f);
      }
    }
    else if(actualSize < 0.80f){
      if(nextSize >= 0.80f){
        optimalSpeed.store(1.0f);
        optimalKp.store(0.50f);
        optimalSpeedTurn.store(1.0f);
        optimalKpTurn.store(0.75f);
      }
      else if(nextSize < 0.80f){
        optimalSpeed.store(1.0f);
        optimalKp.store(0.50f);
        optimalSpeedTurn.store(0.8f);
        optimalKpTurn.store(0.70f);
      }
    }

  }

  void getActualSector(){
    float orientation = heading360.load();
    int thisSector = actualSector.load();
    int thisSectorUpperLimit = sectoresAngs[0][thisSector];
    int thisSectorLowerLimit = sectoresAngs[1][thisSector];
    if(thisSector == 0){
      orientation >= 180? orientation -= 360 : orientation = orientation;

      if(orientation <  -75){
        actualSector.store(3);
        if(driveDirection.load() == 0){
          driveDirection.store(2);
        }
      }
      else if(orientation  > 75){
        actualSector.store(1);
        if(driveDirection.load() == 0){
          driveDirection.store(1);
        }
      }
    }  
    else if(static_cast<int>(orientation) > thisSectorUpperLimit + 30) {
      thisSector++;
      thisSector > 3 ? thisSector = 0 : thisSector = thisSector;
      actualSector.store(thisSector);
    }
    else if(static_cast<int>(orientation) < thisSectorLowerLimit - 30) {
      thisSector--;
      thisSector < 0 ? thisSector = 3 : thisSector = thisSector;
      actualSector.store(thisSector);
    }
  }  

  void getSectorWidth(){
    float orientation = heading360.load();
    if(sectores[0] != 0 && sectores[1] != 0 && sectores[2] != 0 && sectores[3] != 0){
        firstLap.store(false);
    }
    sectores[actualSector.load()] = anchoCorredor.load();
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
    lidarMSG.clear();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float ang = angle_min + angle_inc * static_cast<float>(i);
      if (ang < -0.5235f && ang > -2.6180f || ang > pi) continue; // 0..180°
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      lidarMSG.push_back({ang, pointAngX(ang, r), pointAngY(ang, r), r});
    }
  }

  void on_timer() {
    getOffsetsFromLidar();
    getActualSector();
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
      if(front > 1.3f && front < 1.7f){
        getSectorWidth();
      }
      if(front <= 0.4f){
        returnPWM = controlACDA(0.5f);
        sendAngle = 90 + -angleProccesing( 0.80f, 30.0f);
      }
      else if(front > 0.4f && front <= 1.4f){
        returnPWM = controlACDA(0.8f) - fabs(objectiveAngleVelPD(0.0f, 0.3f));
        sendAngle = 90 + -angleProccesing( 0.70f, 50.0f);
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
    RCLCPP_INFO(this->get_logger(), "primera vuelta");
    auto frame = empaquetar(static_cast<uint16_t>(sendAngle), returnPWM, dir,this->get_logger());
    (void)serial_.write_bytes(frame.data(), frame.size());

    }
  else if(ending == false){
    getOptimalValues();
    if(front <= 1.5f){
      returnPWM = controlACDA(optimalSpeedTurn.load()) - fabs(objectiveAngleVelPD(0.0f, 0.3f));
      sendAngle = 90 + -angleProccesing( optimalKpTurn.load(), 30.0f);
    }
    else{
      returnPWM = controlACDA(optimalSpeed.load() - fabs(objectiveAngleVelPD(0.0f, 0.5f)));
      sendAngle = 90 + -angleProccesing( optimalKp.load(), 50.0f);
    } 
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
    yaw.store(yaw_deg);
    heading360.store(wrap_360(std::fmod(acc, 360.0f)));

    if (msg->twist.twist.linear.z == msg->twist.twist.linear.z) {
      speed.store(msg->twist.twist.linear.z);
    }
    posX_.store(msg->pose.pose.position.x);
    posY_.store(msg->pose.pose.position.y);
    new_otos_data.store(true);
 
  }
  
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr                                    timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            angle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


  //mensaje de rplidar
  
         

  // variables persistentes
  std::atomic<float> optimalSpeedTurn{0.0f};
  std::atomic<float> optimalKpTurn{0.0f};
  std::atomic<float> optimalSpeed{0.0f};
  std::atomic<float> optimalKp{0.0f};
  std::atomic<bool> new_otos_data{false};
  std::atomic<int> actualSector{0};
  std::atomic<int> driveDirection{0}; //0 no determinado, 1 sentido horario, 2 sentido antihorario
  std::atomic<bool>firstLap{true};
  std::atomic<float> anchoCorredor{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<bool> endRound{false};
  std::atomic<bool> init{false};
  std::atomic<int> lastPwm{0};
  std::atomic<float> lastPosX{0.0f};
  std::atomic<float> lastPosY{0.0f};
  std::atomic<float> posX_{0.0f};
  std::atomic<float> posY_{0.0f};
  std::atomic<float> de_f{0.0f};
  std::atomic<float> lastVelErr{0.0f};
  std::atomic<float> lastError{0.0f};
  std::atomic<float> centeringOffset{0.0f};
  std::atomic<float> frontWallDistance{0.0f};
  std::atomic<float> yaw{0.0f};
  std::atomic<float> lastYaw{0.0f};
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
