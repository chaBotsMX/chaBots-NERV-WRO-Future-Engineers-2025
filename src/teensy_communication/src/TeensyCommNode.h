#ifndef TEENSY_COMM_NODE_H
#define TEENSY_COMM_NODE_H

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

#include "SerialPort.h"

struct lidarPoints {
  float angle;  // rad o deg, tú decides
  float x;
  float y;
  float mag;
};

class TeensyCommNode : public rclcpp::Node {
  // Para PID: guardar el error anterior
  float prev_error_ = 0.0f;
  std::chrono::steady_clock::time_point prev_time_ = std::chrono::steady_clock::now();
public:
  TeensyCommNode();

private:
  std::vector<lidarPoints> lidarMSG;
  
  float sectores[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float sectoresAngs[2][4] = {
    {45.0f, 135.0f, 225.0f, 315.0f},
    {315.0f, 45.0f, 135.0f, 225.0f}
  };
 
  SerialPort serial_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
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

  void getOffsetsFromLidar();

  float headingError(const sensor_msgs::msg::LaserScan::SharedPtr msg, float kp);
  float getNextSectorSize();
  float getCurrentSectorSize();
  void getOptimalValues();
  void getActualSector();

  void getSectorWidth();

  static std::array<uint8_t, 6> empaquetar(uint16_t ang_tenths, uint8_t pwm_byte, uint8_t dir, rclcpp::Logger logger);
  void write_to_serial(uint16_t ang_tenths, uint8_t pwm_byte, uint8_t dir, rclcpp::Logger logger);
  
  float clampf(float v, float lo, float hi);

  float feedforward_pwm_multiplier(float targetSpeed);

  int controlACDA(float targetSpeed);

  float angleProccesing(float kpNoLinear = 0.75f, float maxOut = 30.0f);

  float objectiveAngleVelPD(float vel_min, float vel_max);

  //callbacks que se ejecutan al leer un topic
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void on_timer();

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  // --------- Utilidades trig/cartesianas ----------

  inline float wrapPi(float a) {
    while (a <= -M_PI) a += 2.0f*M_PI;
    while (a >   M_PI) a -= 2.0f*M_PI;
    return a;
  }

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

  static inline float wrap_360(float a) { 
    float x = std::fmod(a, 360.0f);
    return (x < 0) ? x + 360.0f : x;
  }

}

#endif
