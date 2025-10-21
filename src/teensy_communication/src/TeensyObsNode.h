#ifndef TEENSY_OBS_NODE_H
#define TEENSY_OBS_NODE_H

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
#include <cerrno>
#include <functional>
#include <algorithm>

#include "SerialPort.h"
#include "Utils.h"

static inline float wrap_360(float a)
{
  float x = std::fmod(a, 360.0f);
  return (x < 0) ? x + 360.0f : x;
}
static inline float wrap_pm180(float a)
{
  float x = std::fmod(a + 180.0f, 360.0f);
  if (x < 0)
    x += 360.0f;
  return x - 180.0f;
}
float wrapError(float a)
{
  if (a > 180.0f)
    return a - 360.0f;
  else if (a < -180.0f)
    return a + 360.0f;
  else
    return a;
}
float grad2rad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }
float rad2grad(float rad) { return rad * 180.0f / static_cast<float>(M_PI); }

static inline float pointAngX(float ang, float r) { return std::cos(ang) * r; }
static inline float pointAngY(float ang, float r) { return std::sin(ang) * r; }
static inline float getDiffAngle(float ang1, float r1, float ang2, float r2)
{
  const float dx = pointAngX(ang2, r2) - pointAngX(ang1, r1);
  const float dy = pointAngY(ang2, r2) - pointAngY(ang1, r1);
  return std::atan2(dy, dx); // rad
}
static inline float getEuclideanDistance(float ang1, float r1, float ang2, float r2)
{
  const float dx = pointAngX(ang2, r2) - pointAngX(ang1, r1);
  const float dy = pointAngY(ang2, r2) - pointAngY(ang1, r1);
  return std::hypot(dx, dy);
}

struct lidarPoints
{
  float angle;
  float x;
  float y;
  float mag;
};

struct Cluster
{
  int type;
  int color;
  int size;
  float start_x;
  float start_y;
  float end_x;
  float end_y;
  float x;
  float y;
};

class TeensyObsNode : public rclcpp::Node
{
public:
  TeensyObsNode() : Node("teensy_obs")
  {
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

private:
  std::vector<Cluster> g_clusters;
  std::vector<lidarPoints> lidarMSG;

  float sectores[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float sectoresAngs[2][4] = {{45.0f, 135.0f, 225.0f, 315.0f},
                              {315.0f, 45.0f, 135.0f, 225.0f}};
  float sectoresTargets[4] = {0.0f, 90.0f, 180.0f, 270.0f};

  static std::array<uint8_t, 6> pack(uint16_t err_deg, uint8_t pwm_byte, uint8_t dir)
  {
    std::array<uint8_t, 6> f{};
    f[0] = 0xAB;
    f[1] = static_cast<uint8_t>((err_deg >> 8) & 0xFF);
    f[2] = static_cast<uint8_t>(err_deg & 0xFF);
    f[3] = pwm_byte; // velocidad
    f[4] = dir;
    uint8_t chk = 0;
    for (int i = 0; i < 5; ++i)
      chk ^= f[i];
    f[5] = chk;
    return f;
  }

  static inline float clampf(float v, float lo, float hi)
  {
    return std::max(lo, std::min(v, hi));
  }
  const float CLUSTER_JUMP_THRESHOLD = 0.20f;
  const float kPI = 3.14159265358979323846f;

  inline float wrapPI(float a)
  {
    while (a <= -M_PI)
      a += 2.0f * M_PI;
    while (a > M_PI)
      a -= 2.0f * M_PI;
    return a;
  }
  void mover(int dir, int pwm, int direction)
  {
    auto frame = pack(dir, pwm, direction);
    (void)serial_.write_bytes(frame.data(), frame.size());
  }

  int controlACDA(float targetSpeed)
  {
    float pwm = 0, jerk = 10;
    float error = targetSpeed - speed_.load();
    float aproxPwm;
    if (targetSpeed < 0.6f)
      aproxPwm = 35.0f;
    else if (targetSpeed < 1.2f)
      aproxPwm = 40.0f;
    else
      aproxPwm = 60.0f;

    float lastPwmLocal = lastPwm_.load();
    float kp = 8.25f;
    float kd = 0.1f;
    pwm = (error * kp) + ((error - lastError_.load()) / 0.01f) * kd;
    pwm = clampf(pwm + aproxPwm, lastPwmLocal - jerk, lastPwmLocal + jerk);
    pwm = clampf(pwm, 0.0f, 255.0f);
    lastPwm_.store(pwm);
    lastError_.store(error);

    if (error < -0.5f || targetSpeed == 0.0f)
      return 0;
    if (error < -0.1f)
      return 1;
    return static_cast<int>(pwm);
  }

  // ---- Orientation and position update with OTOS ----
  void getActualSector()
  {
    float orientation = heading360_.load();
    int thisSector = actualSector.load();
    int thisSectorUpperLimit = sectoresAngs[0][thisSector];
    int thisSectorLowerLimit = sectoresAngs[1][thisSector];
    if (thisSector == 0)
    {
      orientation >= 180 ? orientation -= 360 : orientation = orientation;

      if (orientation < -50)
      {
        actualSector.store(3);
        if (direction_.load() == 0)
        {
          direction_.store(2);
        }
      }
      else if (orientation > 50)
      {
        actualSector.store(1);
        if (direction_.load() == 0)
        {
          direction_.store(1);
        }
      }
    }
    else if (static_cast<int>(orientation) > thisSectorUpperLimit + 5)
    {
      thisSector++;
      thisSector > 3 ? thisSector = 0 : thisSector = thisSector;
      actualSector.store(thisSector);
    }
    else if (static_cast<int>(orientation) < thisSectorLowerLimit - 5)
    {
      thisSector--;
      thisSector < 0 ? thisSector = 3 : thisSector = thisSector;
      actualSector.store(thisSector);
    }
  }

  int getDriveDir()
  {
    if (absolute_angle_.load() < 90.0f && dist_front_.load() < 0.35f)
    {
      return 1; // izquierda
    }
    else if (absolute_angle_.load() > 90.0f && dist_front_.load() < 0.35f)
    {
      return 2; // derecha
    }
    else
    {
      return 0;
    }
  }

  // ---- LIDAR procesing ---
  void updateLidarwithOtos()
  {
    const float yaw_prev = grad2rad(lastYaw.load());
    const float dx_w = posX_.load() - lastPosX.load();
    const float dy_w = posY_.load() - lastPosY.load();
    const float dth = wrapPI(grad2rad(yaw.load() - lastYaw.load()));

    const float c0 = std::cos(yaw_prev), s0 = std::sin(yaw_prev);
    const float dx_b = c0 * dx_w + s0 * dy_w;
    const float dy_b = -s0 * dx_w + c0 * dy_w;

    const float c = std::cos(-dth), s = std::sin(-dth);

    for (auto &spt : lidarMSG)
    {
      float x = spt.x - dx_b;
      float y = spt.y - dy_b;

      float xr = c * x - s * y;
      float yr = s * x + c * y;

      spt.x = xr;
      spt.y = yr;
      spt.angle = wrapPI(std::atan2(yr, xr));
      spt.mag = std::hypot(xr, yr);
    }

    lastPosX.store(posX_.load());
    lastPosY.store(posY_.load());
    lastYaw.store(yaw.load());
  }
  // ---- Clustering LIDAR points ----
  void find_clusters()
  {
    g_clusters.clear();
    const int n = static_cast<int>(lidarMSG.size());
    if (n == 0)
      return;

    int start = 0;               // índice de inicio del clúster actual
    double sumx = lidarMSG[0].x; // acumuladores para centroide
    double sumy = lidarMSG[0].y;
    int count = 1;

    for (int i = 1; i < n; ++i)
    {
      // salto euclidiano entre puntos consecutivos (usando tus ángulos y magnitudes)
      float jump = Utils::getEuclideanDistance(
          lidarMSG[i - 1].angle, lidarMSG[i - 1].mag,
          lidarMSG[i].angle, lidarMSG[i].mag);

      if (jump > CLUSTER_JUMP_THRESHOLD)
      {
        // cerramos clúster [start .. i-1]
        Cluster c{};
        c.type = 0;
        c.color = 0;
        c.size = count;
        c.start_x = lidarMSG[start].x;
        c.start_y = lidarMSG[start].y;
        c.end_x = lidarMSG[i - 1].x;
        c.end_y = lidarMSG[i - 1].y;
        c.x = static_cast<float>(sumx / count);
        c.y = static_cast<float>(sumy / count);
        g_clusters.push_back(c);

        // comenzamos nuevo clúster en i
        start = i;
        sumx = lidarMSG[i].x;
        sumy = lidarMSG[i].y;
        count = 1;
      }
      else
      {
        // seguimos en el mismo clúster
        sumx += lidarMSG[i].x;
        sumy += lidarMSG[i].y;
        ++count;
      }
    }

    // cerrar el último clúster (hasta n-1)
    if (count > 0)
    {
      Cluster c{};
      c.type = 0;
      c.color = 0;
      c.size = count;
      c.start_x = lidarMSG[start].x;
      c.start_y = lidarMSG[start].y;
      c.end_x = lidarMSG.back().x;
      c.end_y = lidarMSG.back().y;
      c.x = static_cast<float>(sumx / count);
      c.y = static_cast<float>(sumy / count);
      g_clusters.push_back(c);
    }
  }

  void classify_clusters()
  {
    int size = static_cast<int>(g_clusters.size());
    for (int i = 0; i < size; ++i)
    {
      float cluster_size = getEuclideanDistance(g_clusters[i].start_x, g_clusters[i].start_y,
                                                g_clusters[i].end_x, g_clusters[i].end_y);
      if (cluster_size < 0.07f)
      {
        g_clusters[i].type = 1; // bloque
      }
      else
      {
        g_clusters[i].type = 0; // pared
      }
    }
  }
  void take_measures()
  {
    // inicializa a “muy lejos”
    float best_front = std::numeric_limits<float>::infinity();
    float best_side = std::numeric_limits<float>::infinity();

    float best_block = std::numeric_limits<float>::infinity();
    float bx = 0.0f, by = 0.0f;

    for (const auto &c : g_clusters)
    {
      if (c.type == 0)
      { // pared
        // frente = distancia sobre eje X (tu 0 rad mira al +X)
        best_front = std::min(best_front, std::fabs(c.x));
        // lateral (pared exterior) = distancia sobre eje Y
        best_side = std::min(best_side, std::fabs(c.y));
      }
      else
      {                                 // bloque
        float d = std::hypot(c.x, c.y); // distancia cartesiana correcta
        if (d < best_block)
        {
          best_block = d;
          bx = c.x;
          by = c.y;
        }
      }
    }

    if (std::isfinite(best_front))
      distance_to_front_wall.store(best_front);
    if (std::isfinite(best_side))
      distance_to_out_wall.store(best_side);
    if (std::isfinite(best_block))
    {
      obj_avoid_dis.store(best_block);
      obj_avoid_x.store(bx);
      obj_avoid_y.store(by);
    }
  }

  // ---- callbacks ----

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const float angle_min = msg->angle_min;
    const float angle_inc = msg->angle_increment;
    const float pi = static_cast<float>(M_PI);
    lidarMSG.clear();

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      const float ang = angle_min + angle_inc * static_cast<float>(i);
      if (ang < 0.0f || ang > 3.14159f)
        continue;
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
        continue;
      lidarMSG.push_back({ang, pointAngX(ang, r), pointAngY(ang, r), r});
    }
    find_clusters();
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &q = msg->pose.pose.orientation;
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0f / kPI;

    static bool init = false;
    static float prev = 0.0f;
    static float acc = 0.0f;
    if (!init)
    {
      prev = yaw_deg;
      acc = 0.0f;
      init = true;
    }
    else
    {
      float d = yaw_deg - prev;
      if (d > 180.0f)
        d -= 360.0f;
      if (d < -180.0f)
        d += 360.0f;
      acc += d;
      prev = yaw_deg;
    }

    yaw.store(yaw_deg);
    heading_acc_.store(acc);
    heading360_.store(wrap_360(acc));

    if (!std::isnan(msg->twist.twist.linear.z))
    {
      speed_.store(msg->twist.twist.linear.z);
    }

    posX_.store(msg->pose.pose.position.x);
    posY_.store(msg->pose.pose.position.y);
    new_otos_data.store(true);
  }

  void on_timer()
  {

    if (new_otos_data.load())
    {
      updateLidarwithOtos();
      new_otos_data.store(false);
    }
    //  getOffSetsFromLidar();
    getActualSector();
    int numberClusters = static_cast<int>(g_clusters.size());
    RCLCPP_INFO(this->get_logger(), "Numero de clusters: %d", numberClusters);
    for (int i = 0; i < numberClusters; ++i)
    {
      RCLCPP_INFO(this->get_logger(), "Cluster %d: Tipo=%d, Tamaño=%d, Centro=(%.2f, %.2f)", i,
                  g_clusters[i].type, g_clusters[i].size, g_clusters[i].x, g_clusters[i].y);
    }
  }

  void on_objects_detections(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto &data = msg->data;
    size_t n = data.size() / 3;
    if (n == 0)
    {
      object_status_.store(0.0f);
      return;
    }

    float min_dist = std::numeric_limits<float>::max();
    int idx_min = -1;
    for (size_t i = 0; i < n; ++i)
    {
      float dist = data[i * 3 + 1];
      if (dist < min_dist)
      {
        min_dist = dist;
        idx_min = static_cast<int>(i);
      }
    }
    if (idx_min >= 0)
    {
      object_color_.store(data[idx_min * 3 + 0]);
      object_distance_.store(data[idx_min * 3 + 1]);
      object_angle_.store(data[idx_min * 3 + 2]);
      object_status_.store(1.0f);
    }
    else
    {
      object_status_.store(0.0f);
    }
  }

  // ---- miembros ----
  SerialPort serial_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr objects_detections_sub_;

  // atómicos (siempre .load() / .store())
  std::atomic<float> obj_avoid_dis{0.0f};
  std::atomic<float> obj_avoid_x{0.0f};
  std::atomic<float> obj_avoid_y{0.0f};
  std::atomic<float> distance_to_front_wall{0.0f};
  std::atomic<float> distance_to_out_wall{0.0f};
  std::atomic<float> lastYaw{0.0f};
  std::atomic<float> yaw{0.0f};
  std::atomic<bool> new_otos_data{false};
  std::atomic<float> posX_{0.0f};
  std::atomic<float> posY_{0.0f};
  std::atomic<float> lastPosX{0.0f};
  std::atomic<float> lastPosY{0.0f};
  std::atomic<float> yaw_{0.0f};
  std::atomic<float> lastYaw_{0.0f};
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
  std::atomic<int> cycle_idx_{0};
  std::atomic<bool> last_blocked_{false};
};

#endif