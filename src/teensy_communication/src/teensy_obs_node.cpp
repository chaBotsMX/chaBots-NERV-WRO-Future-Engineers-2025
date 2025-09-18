#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

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
float wrapError(float a){
  if (a > 180.0f) return a - 360.0f;
  else if (a < -180.0f) return a + 360.0f;
  else return a;
}

class TeensyObsNode : public rclcpp::Node {
public:
  TeensyObsNode() : Node("teensy_obs") {
    (void)serial_.open_port();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&TeensyObsNode::on_scan, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&TeensyObsNode::on_odom, this, std::placeholders::_1));

    // Suscripción al arreglo de detecciones de objetos
    objects_detections_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/objects/detections", 10,
      std::bind(&TeensyObsNode::on_objects_detections, this, std::placeholders::_1));

    // Estado inicial         // 0°
    cycle_idx_.store(0);
    last_blocked_.store(false);

    timer_ = create_wall_timer(10ms, std::bind(&TeensyObsNode::on_timer, this));
  }

  void on_objects_detections(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    const auto& data = msg->data;
    size_t n = data.size() / 3;
    if (n == 0) {
      object_status_.store(0.0f);
      return;
    }

    float min_dist = std::numeric_limits<float>::max();
    int idx_min = -1;
    for (size_t i = 0; i < n; ++i) {
      float dist = data[i * 3 + 1];
      if (dist < min_dist) {
        min_dist = dist;
        idx_min = static_cast<int>(i);
      }
    }
    if (idx_min >= 0) {
      object_color_.store(data[idx_min * 3 + 0]);
      object_distance_.store(data[idx_min * 3 + 1]);
      object_angle_.store(data[idx_min * 3 + 2]);
      object_status_.store(1.0f);
    } else {
      object_status_.store(0.0f);
    }
  }

private:


  float sectores[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float sectoresAngs[2][4] = {{45.0f, 135.0f, 225.0f, 315.0f},
                              {315.0f, 45.0f, 135.0f, 225.0f}};
  float sectoresTargets[4] = {0.0f, 90.0f, 180.0f, 270.0f};

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


  void mover(int dir, int pwm, int direction){
    auto frame = pack(dir, pwm, direction);
    (void)serial_.write_bytes(frame.data(), frame.size());
  }
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

    float sumX = 0.0f; float sumY = 0.0f;
    float sumFront = 0.0f; size_t totalFront = 0;
    float sumLeft = 0.0f; size_t totalLeft = 0;
    float sumRight = 0.0f; size_t totalRight = 0;
    float sumBack = 0.0f; size_t totalBack = 0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float a = a_min + a_inc * static_cast<float>(i);
      if (a < 0.0f && a > -1.3962f || a < -1.7453f || a > 3.141592f ) continue; 
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      if(a> 0.0f && a < 3.1415f) {
        sumX += r * std::cos(a);
        sumY += r * std::sin(a);
      }
      if(a < -1.3962f && a > -1.7453f) { sumBack += r; ++totalBack; }
      if (a > 1.39 && a < 1.7453f) { sumFront += r; ++totalFront; }
      if( a > 0.0f && a < 0.5235f) { sumLeft += r; ++totalLeft; }
      if( a > 2.79252f && a < 3.141592f) { sumRight += r; ++totalRight; }
    }
    absolute_angle_.store(std::atan2(sumY, sumX) * 180.0f / kPI);
    const float backmean = (totalBack ? (sumBack / static_cast<float>(totalBack)) : std::numeric_limits<float>::quiet_NaN());
    dist_back_.store(backmean);
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


  void getActualSector(){
    float orientation = heading360_.load();
    int thisSector = actualSector.load();
    int thisSectorUpperLimit = sectoresAngs[0][thisSector];
    int thisSectorLowerLimit = sectoresAngs[1][thisSector];
    if(thisSector == 0){
      orientation >= 180? orientation -= 360 : orientation = orientation;

      if(orientation <  -70){
        actualSector.store(3);
        if(direction_.load() == 0){
          direction_.store(2);
        }
      }
      else if(orientation  > 70){
        actualSector.store(1);
        if(direction_.load() == 0){
          direction_.store(1);
        }
      }
    }  
    else if(static_cast<int>(orientation) > thisSectorUpperLimit+15) {
      thisSector++;
      thisSector > 3 ? thisSector = 0 : thisSector = thisSector;
      actualSector.store(thisSector);
    }
    else if(static_cast<int>(orientation) < thisSectorLowerLimit) {
      thisSector--;
      thisSector < 0 ? thisSector = 3 : thisSector = thisSector-15;
      actualSector.store(thisSector);
    }
  }  


  
  void orientar(){
    float offset = heading360_.load();
    float target = targetYaw_.load();
    float err = wrap_pm180(target - offset);

    float returnCorrection = std::clamp(90.0f + (err * 1.0f), 40.0f, 160.0f);

    RCLCPP_INFO(this->get_logger(), "Offset: %f, Target: %f, Error: %f, Correction: %f", offset, target, err, returnCorrection);
    auto frame = pack(static_cast<int>(returnCorrection), 40, 0);
    (void)serial_.write_bytes(frame.data(), frame.size());
  }
  int getDriveDir(){
    if(absolute_angle_.load() < 90.0f){
      return 1; // izquierda
    }
    else if(absolute_angle_.load() > 90.0f){
      return 2; // derecha
    }
    else{
      return 0; 
    }
  }
  void girar(){
    if(direction_.load() == 0 ){
      RCLCPP_INFO(this->get_logger(), "Decidiendo dirección de giro, direccion actual: %d", direction_.load());
      direction_.store(getDriveDir());
    }
    if(direction_.load() == 2 && turnAllowed_.load()){
      targetYaw_.store(wrap_360(targetYaw_.load() + 90.0f));
      turnAllowed_.store(false);
      inturn.store(true);
      for(auto &i : turnStep) i.store(false); 
    }
    else if(direction_.load() == 1 && turnAllowed_.load()){
      targetYaw_.store(wrap_360(targetYaw_.load() - 90.0f));
      turnAllowed_.store(false);
      inturn.store(true);
    }

  }

  void rutinaGirar(){
    float distanFront = dist_front_.load();
    float distBack = dist_back_.load();
    if(!std::isfinite(distanFront)) {
      distanFront = 0.0f;
    }
    if(!std::isfinite(distBack)) {
      distBack = 0.0f;
    }
    float outWallDistance = 0;
    if(direction_.load() == 1){
      RCLCPP_INFO(this->get_logger(), "Distancia pared derecha: %f", dist_Right_.load());
      outWallDistance = dist_Right_.load();
    }
    else if(direction_.load() == 2){
      RCLCPP_INFO(this->get_logger(), "Distancia pared izquierda: %f", dist_Left_.load());
      outWallDistance = dist_Left_.load();
    }
    RCLCPP_INFO(this->get_logger(), "Distancia pared exterior: %f", outWallDistance);
    if(turntype_.load() == 0){
        if(outWallDistance >= 0.60f){
          turntype_.store(3);
        }
        else if(outWallDistance < 0.60f && outWallDistance >= 0.45f){
          turntype_.store(2);
        }
        else if(outWallDistance < 0.45f){
          turntype_.store(1);
        }
    }

    if(turntype_.load() == 1){
      RCLCPP_INFO(this->get_logger(), "Giro tipo 1"); 
      if(turnStep[0].load() == false){
        float correction = wrap_pm180(targetYaw_.load() - heading360_.load());
        mover(90 + correction, 40, 0);
        if(fabs(correction) < 20.0f){
          turnStep[0].store(true);
        }
      }
      else if(turnStep[1].load() == false){
        mover(90, 40, 1);
        if(distBack < 0.5f){
          turnStep[1].store(true);
          inturn.store(false);
          turntype_.store(0);
          for(int i = 0; i < 4; i++){
            turnStep[i].store(false);
          }
        }
      }
    }
    else if(turntype_.load() == 2){
      const char* prompt = (turnStep[0].load() ? "Giro tipo 2 yes" : "Giro tipo 2 no");
      RCLCPP_INFO(this->get_logger(), prompt);
      if(turnStep[0].load() == false){
        RCLCPP_INFO(this->get_logger(), "Paso 1");
        mover(90,30,0);
        if(distanFront < 0.4f){
          turnStep[0].store(true);
        }
      }
      else if(turnStep[1].load() == false){
        RCLCPP_INFO(this->get_logger(), "Paso 2");
        float correction = wrap_pm180((targetYaw_.load() - 45) - heading360_.load());
        mover(90 + correction, 40, 0);
        if(fabs(correction) < 20.0f){
          turnStep[1].store(true);
        }
      }
      else if(turnStep[2].load() == false){
        float correction = wrap_pm180(targetYaw_.load() - heading360_.load());
        mover(90 - correction, 40, 1);
        if(fabs(correction) < 10.0f){
          turnStep[2].store(true);
        }
      }
      else if(turnStep[3].load() == false){ 
        mover(90, 40, 1); //giro parte 2
        if(distBack < 0.5f){
          turnStep[3].store(true);
          inturn.store(false);
          turntype_.store(0);
          for(int i = 0; i < 4; i++){
            turnStep[i].store(false);
          }
        }
      }
    } 


  else if(turntype_.load() == 3){
      RCLCPP_INFO(this->get_logger(), "Giro tipo 3");
      if(turnStep[0].load() == false){
        RCLCPP_INFO(this->get_logger(), "Paso 1");
        mover(90, 40, 0); //giro parte 1
        if(distanFront < 0.30f){
          turnStep[0].store(true);
        }
      }
      else if(turnStep[1].load() == false){
        RCLCPP_INFO(this->get_logger(), "Paso 2");
        if(direction_.load() == 1) mover(150, 40, 1);
        else if(direction_.load() == 2) mover(30, 40, 1);
        if(fabs(targetYaw_.load() - heading360_.load()) < 5.0f){
          turnStep[1].store(true);
        }
      }
      else if(turnStep[2].load() == false){
        mover(90, 40, 1); 
        if(distBack < 0.5f){
          turnStep[2].store(true);
          inturn.store(false);
          turntype_.store(0);
          for(int i = 0; i < 4; i++){
            turnStep[i].store(false);
          }
        }
      }
    }
  }

  void on_timer() {
    if (!std::isfinite(heading360_.load())) return;
    getActualSector();
      int isObs = object_status_.load();
      int sector = actualSector.load();

      if(sector != lastSector.load()){
        lastSector.store(sector);
        turnAllowed_.store(true);
        targetYaw_.store(sectoresTargets[actualSector.load()]);
        //RCLCPP_INFO(this->get_logger(), "Nuevo sector: %d", sector);
      }
      RCLCPP_INFO(this->get_logger(), "Obstaculo detectado: distancia al frente: %f, orientacion: %f", dist_front_.load(), heading360_.load());
      if(inturn.load()){
        rutinaGirar();
        return;
      }
      else if(isObs == 1){ 
        float angle = object_angle_.load();
        float object_distance = object_distance_.load() /2;
        int color = static_cast<int>(object_color_.load());
        float cubeSectorAngle = 0;

        if(object_distance < 30.0f){
          if(!wasClose_.load()){
            RCLCPP_INFO(this->get_logger(), "Cubo cerca: distancia al cubo: %f, orientacion: %f", object_distance, heading360_.load());
            wasClose_.store(true);
          }
        } 
        if(color != last_color_.load() || last_Cube_Distance_.load() - object_distance < -10.0f && wasClose_.load()){
          RCLCPP_INFO(this->get_logger(), "Color del cubo cambiado a: %d", color);
          cube_target_changued_.store(true);
          last_color_.store(color);
          wasClose_.store(false);
          last_Cube_Distance_.store(object_distance);
        }
        else{
          last_Cube_Distance_.store(object_distance);
          last_color_.store(color);
        }
        if(cube_target_changued_.load()){
          float cubeAbsAngle = wrap_360(heading360_.load() - angle);
          if(cubeAbsAngle < 45){
            cubeSector_.store(0);
          }
          else if(cubeAbsAngle >= 45 && cubeAbsAngle < 135){
            cubeSector_.store(1);
          }
          else if(cubeAbsAngle >= 135 && cubeAbsAngle < 225){
            cubeSector_.store(2);
          }
          else if(cubeAbsAngle >= 225 && cubeAbsAngle < 315){
            cubeSector_.store(3);
          }
          else if(cubeAbsAngle >= 315){
            cubeSector_.store(0);
          }
          actualSector.load() != cubeSector_.load() ? actualSector.store(cubeSector_.load()) : actualSector.store(actualSector.load());
          cube_target_changued_.store(false);

        }

        if(dist_front_.load() - (object_distance/100) < 0.80f && fabs(angle) > 10.0f  && fabs(wrapError(targetYaw_.load() - heading360_.load())) < 7.0f){
          RCLCPP_INFO(this->get_logger(), "Obstáculo muy cerca, orientando antes de maniobrar: %f a %f grados", dist_front_.load() - (object_distance/100), angle);
          orientar();
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Cubo detectado: distancia al cubo: %f, orientacion al cubo: %f, color: %d, sector del cubo: %d, sector actual: %d", object_distance, angle, color, cubeSector_.load(), actualSector.load());
        if (color == 0) { // VERDE => pasar SIEMPRE por la IZQUIERDA si está a la izquierda; derecha solo si bloquea
          constexpr float minDis = 30.0f;   // cm
          constexpr float maxDis = 100.0f;  // cm
          constexpr float OffSetmax   = 30.0f;   // ° de desvío máximo (hacia IZQ)
          constexpr float tickMaxChange = 3.0f;    // °/tick, límite de cambio (anti-jerk)
          constexpr float safe = 40.0f;   // °, medio ángulo del cono frontal (para "bloquea" en derecha)

          static float servo = 90.0f;     // estado para suavizado
          float compX = std::cos(angle);
          float disX = object_distance * compX;
          RCLCPP_INFO(this->get_logger(), "Componente X del ángulo del cubo: %f", compX);

          if(disX < 0.0f){
            auto frame = pack(static_cast<uint16_t>((cmd_raw)), pwm, 0);
            (void)serial_.write_bytes(frame.data(), frame.size());  
            RCLCPP_INFO(this->get_logger(),
              "Evasion IZQ | d=%.1fcm w_d=%.2f | ang=%.1f° lado=%s | off=%.1f° cmd=%.1f° cmd_raw=%.1f° errHelper=%.1f° | distLeft=%.2fcm",
              dis, prop, angle, (angle < 0.0f ? "izq(always)" : (theta <= safe ? "der(block)" : "der(free)")),
              offset, cmd_deg, cmd_raw, errHelper, dist_Left_.load());
          }
        }

        else if (color == 1) { // ROJO => pasar por la DERECHA (contrario a verde)

        // --- Parámetros (usa mismos que en VERDE para tuning coherente) ---
        constexpr float minDis        = 30.0f;   // cm
        constexpr float maxDis        = 100.0f;  // cm
        constexpr float OffSetmax     = 30.0f;   // ° de desvío máximo (hacia DER)
        constexpr float tickMaxChange = 3.0f;    // °/tick (anti-jerk)
        constexpr float safe          = 40.0f;   // °, medio ángulo del cono frontal

        // --- Estado para suavizado ---
        static float servo = 90.0f;

        // --- Lecturas y pesos ---
        float dis   = clampf(object_distance, minDis, maxDis);
        float prop  = (maxDis - dis) / (maxDis - minDis);           // [0..1] más cerca => mayor
        float inv_prop = (dis / maxDis) * 0.375f;
        float theta = std::fabs(angle);                              // magnitud angular (°)

        // --- Decisión: derecha siempre; izquierda solo si bloquea ---
        bool  must_evade = false;
        float offset     = 0.0f;   // positivo = DERECHA

        if (angle < 0.0f) {
          // LADO DERECHO: evadir SIEMPRE
          must_evade = true;
          offset     = - OffSetmax * prop;                           // [0, +OffSetmax]
        } else {
          // LADO IZQUIERDO: evadir SOLO si bloquea (dentro del cono frontal)
          if (theta <= safe) {
            float w_phi = std::pow(std::cos((kPI * 0.5f) * (theta / safe)), 2.0f); // [0..1]
            must_evade  = true;
            offset      = - OffSetmax * prop * w_phi;                // [0, +OffSetmax]
          }
        }

        if (!must_evade || prop <= 0.0f || !std::isfinite(dis) || !std::isfinite(theta)) {
          orientar();
        } else {
          // --- Suavizado del mando ---
          float cmd_raw = clampf(90.0f + offset + wrap_pm180((targetYaw_.load() - heading360_.load()) * inv_prop),60,150);                            // servo centrado en 90°
          float delta   = clampf(cmd_raw - servo, -tickMaxChange, tickMaxChange);
          float cmd_deg = servo + delta;
          servo = cmd_deg;

          const uint8_t pwm = 40; // opcional: escalar con prop y |offset|

          auto frame = pack(static_cast<uint16_t>(std::lround(cmd_deg)), pwm, 0);
          (void)serial_.write_bytes(frame.data(), frame.size());
          RCLCPP_INFO(this->get_logger(),
            "Evasion DER | d=%.1fcm prop=%.2f | ang=%.1f° lado=%s | off=%.1f° cmd=%.1f°",
            dis, prop, angle, (angle > 0.0f ? "der(always)" : (theta <= safe ? "izq(block)" : "izq(free)")),
            offset, cmd_deg);
        }
      }


        else{
          orientar();
          RCLCPP_INFO(this->get_logger(), "Obstaculo no identificado, orientando");
          if(dist_front_.load() < 1.0f && fabs(targetYaw_.load() - heading360_.load()) < 7.0f){
            RCLCPP_INFO(this->get_logger(), "cambio de objetivo");
            girar();
          }
        }

      }
      else{
        orientar();
        RCLCPP_INFO(this->get_logger(), "No hay obstaculo, orientando");
        if(dist_front_.load() < 1.0f && fabs(targetYaw_.load() - heading360_.load()) < 7.0f){
          RCLCPP_INFO(this->get_logger(), "cambio de objetivo");
          girar();
        }
      }
    }

  


// ---- miembros ----
  SerialPort serial_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr objects_detections_sub_;

  // atómicos (siempre .load() / .store())
  std::atomic<bool> wasClose_{false};
  std::atomic<bool> cube_target_changued_{false};
  std::atomic<float> last_Cube_Distance_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<int> cubeSector_{-1};
  std::atomic<int> last_color_{-1};
  std::atomic<bool> turnStep[4] = {false, false, false, false};
  std::atomic<int> turntype_{0}; // 1 = close turn, 2 = middle turn , 3 = far turn
  std::atomic<bool> inturn{false};
  std::atomic<bool> out_{false};
  std::atomic<bool> initCorrection_{false};
  std::atomic<bool> repeat_{false};
  std::atomic<bool> changueDelay_{false};
  std::atomic<bool> frontInParking_{false};
  std::atomic<bool> backInParking_{false};
  std::atomic<bool> outOfParking_{false};
  std::atomic<int> actualSector{0};
  std::atomic<int> lastSector{0};
  std::atomic<float> targetYaw_{0.0f};
  std::atomic<int> direction_{0}; // 1 = "LEFT", 2 = "RIGHT"
  std::atomic<bool> turnAllowed_{true}; 
  std::atomic<float> dist_back_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> dist_Left_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> dist_Right_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> poseY_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> speed_{0.0f};
  std::atomic<float> lastPwm_{0.0f};
  std::atomic<float> lastError_{0.0f};
  std::atomic<float> absolute_angle_{0.0f};
  std::atomic<float> heading_acc_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> heading360_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> dist_front_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> object_distance_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> object_angle_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> object_color_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> object_status_{0.0f};
  std::atomic<int>   cycle_idx_{0};
  std::atomic<bool>  last_blocked_{false};

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::this_thread::sleep_for(std::chrono::seconds(5));
  rclcpp::spin(std::make_shared<TeensyObsNode>());
  rclcpp::shutdown();
  return 0;
}