#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <chrono>
#include <atomic>
#include <array>
#include <mutex>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <cstring>

using namespace std::chrono_literals;

// --------------------- Serial helper ---------------------
class SerialPort {
public:
  SerialPort() : fd_(-1) {}
  ~SerialPort() { close_port(); }

  bool open_port(const std::string &port, int baudrate) {
    close_port();
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) { last_error_ = std::string("open() failed: ") + std::strerror(errno); return false; }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) { last_error_ = std::string("tcgetattr() failed: ")+ std::strerror(errno); close_port(); return false; }
    cfmakeraw(&tio);

    speed_t speed = B115200;
    switch (baudrate) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default: speed = B115200; break;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB; // 8N1
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) { last_error_ = std::string("tcsetattr() failed: ") + std::strerror(errno); close_port(); return false; }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close_port() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
  bool is_open() const { return fd_ >= 0; }

  bool write_bytes(const uint8_t* data, size_t len) {
    if (fd_ < 0) return false;
    size_t total = 0;
    while (total < len) {
      ssize_t n = ::write(fd_, data + total, len - total);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) { usleep(1000); continue; }
        last_error_ = std::string("write() failed: ") + std::strerror(errno);
        return false;
      }
      total += static_cast<size_t>(n);
    }
    return true;
  }

  const std::string& last_error() const { return last_error_; }

private:
  int fd_;
  std::string last_error_;
};

// --------------------- Node ---------------------
class TeensyCommNode : public rclcpp::Node {
public:
  TeensyCommNode() : Node("teensy_comm") {
    // ---- Params ----
    port_        = declare_parameter<std::string>("port", "/dev/ttyAMA0");
    baud_        = declare_parameter<int>("baud", 115200);
    rate_hz_     = declare_parameter<int>("rate_hz", 200);

    // LCP tuning
    k_perp_      = declare_parameter<double>("k_perp",      0.9);
    look_L_      = declare_parameter<double>("look_L",      0.35);
    ema_alpha_   = declare_parameter<double>("ema_alpha",   0.15);
    trim_ratio_  = declare_parameter<double>("trim_ratio",  0.2);

    // Gate
    gate_speed_min_    = declare_parameter<double>("gate_speed_min",   0.05);
    gate_yaw_tol_deg_  = declare_parameter<double>("gate_yaw_tol_deg", 35.0);
    gate_cooldown_s_   = declare_parameter<double>("gate_cooldown_s",  2.0);
    target_laps_       = declare_parameter<int>("target_laps", 3);

    // Return-to-zone (1m box)
    zone_side_         = declare_parameter<double>("zone_side",   1.0);
    zone_margin_       = declare_parameter<double>("zone_margin", 0.08);
    return_speed_      = declare_parameter<double>("return_speed",0.20);
    hold_time_         = declare_parameter<double>("hold_time",   0.7);
    stop_speed_thresh_ = declare_parameter<double>("stop_speed_thresh", 0.05);
    guard_dist_        = declare_parameter<double>("guard_dist",  0.18); // prefer LCP if closer than this to a wall

    // Serial open
    RCLCPP_INFO(get_logger(), "Opening serial: %s @ %d", port_.c_str(), baud_);
    if (!serial_.open_port(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "Serial open failed: %s", serial_.last_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial opened.");
    }

    // Publ (debug)
    angle_pub_ = create_publisher<std_msgs::msg::Float32>("/lcp_angle_deg", 10);

    // Subs
    control_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/control_params", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
        last_vel_mps_.store(static_cast<float>(msg->x));
        last_kp_.store(static_cast<float>(msg->y));
      }
    );

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&TeensyCommNode::on_scan, this, std::placeholders::_1)
    );

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TeensyCommNode::on_odom, this, std::placeholders::_1)
    );

    // Timer send
    const int period_ms = std::max(1, 1000 / std::max(1, rate_hz_));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&TeensyCommNode::on_timer, this));
  }

private:
  enum class Mode { RUN_LCP, RETURN_TO_ZONE, DONE };

  // 8B frame: [0xAB][ANG_H][ANG_L][VEL_H][VEL_L][KP_H][KP_L][CHK]
  static std::array<uint8_t, 8> make_frame_v2(uint16_t ang_tenths, uint16_t vel_mmps, uint16_t kp_milli) {
    std::array<uint8_t, 8> f{
      0xAB,
      static_cast<uint8_t>((ang_tenths >> 8) & 0xFF),
      static_cast<uint8_t>(ang_tenths & 0xFF),
      static_cast<uint8_t>((vel_mmps >> 8) & 0xFF),
      static_cast<uint8_t>(vel_mmps & 0xFF),
      static_cast<uint8_t>((kp_milli >> 8) & 0xFF),
      static_cast<uint8_t>(kp_milli & 0xFF),
      0x00
    };
    uint8_t chk = 0x00;
    for (size_t i = 0; i < 7; ++i) chk ^= f[i];
    f[7] = chk;
    return f;
  }

  static uint16_t clamp_u16(int v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return static_cast<uint16_t>(v);
  }

  static float wrap_deg_pm180(float a) {
    while (a > 180.f) a -= 360.f;
    while (a <= -180.f) a += 360.f;
    return a;
  }
  static float wrap_deg_0_360(float a) {
    while (a >= 360.f) a -= 360.f;
    while (a < 0.f)    a += 360.f;
    return a;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr m) {
    const auto &p = m->pose.pose.position;
    odom_x_.store(static_cast<float>(p.x));
    odom_y_.store(static_cast<float>(p.y));

    const auto &q = m->pose.pose.orientation;
    float siny_cosp = 2.f*(q.w*q.z + q.x*q.y);
    float cosy_cosp = 1.f - 2.f*(q.y*q.y + q.z*q.z);
    float yaw = std::atan2(siny_cosp, cosy_cosp); // rad
    odom_yaw_deg_.store(wrap_deg_0_360(static_cast<float>(yaw * 180.0 / M_PI)));

    float vx = static_cast<float>(m->twist.twist.linear.x);
    float vy = static_cast<float>(m->twist.twist.linear.y);
    odom_speed_.store(std::hypot(vx, vy));

    if (!gate_init_) {
      C0x_ = p.x; C0y_ = p.y; // start zone center
      float psi_use_deg = std::isfinite(psi_corr_deg_.load()) ? psi_corr_deg_.load() : odom_yaw_deg_.load();
      float pr = psi_use_deg * static_cast<float>(M_PI) / 180.0f;
      bx_ = std::cos(pr); by_ = std::sin(pr);
      float n = std::hypot(bx_, by_); if (n>1e-6f) { bx_/=n; by_/=n; }
      last_g_ = bx_*(odom_x_.load()-C0x_) + by_*(odom_y_.load()-C0y_);
      gate_init_ = true;
      last_lap_stamp_ = now();
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const float a_min = msg->angle_min;
    const float a_inc = msg->angle_increment;
    const float zero  = 0.0f;
    const float pi    = static_cast<float>(M_PI);

    std::vector<std::array<float,2>> pts;
    pts.reserve(msg->ranges.size()/2);
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float ang = a_min + a_inc * static_cast<float>(i);
      if (ang < zero || ang > pi) continue;
      float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      float x = r * std::cos(ang);
      float y = r * std::sin(ang);
      pts.push_back({x,y});
    }
    if (pts.size() < 20) return;

    // PCA
    double sx=0.0, sy=0.0;
    for (auto &p: pts){ sx+=p[0]; sy+=p[1]; }
    double invN = 1.0 / static_cast<double>(pts.size());
    double mx = sx*invN, my = sy*invN;
    double cxx=0.0, cxy=0.0, cyy=0.0;
    for (auto &p: pts){
      double dx=p[0]-mx, dy=p[1]-my;
      cxx+=dx*dx; cxy+=dx*dy; cyy+=dy*dy;
    }
    cxx*=invN; cxy*=invN; cyy*=invN;
    double T=cxx+cyy, D=cxx*cyy - cxy*cxy;
    double disc = std::max(0.0, T*T/4.0 - D);
    double lam1 = T/2.0 + std::sqrt(disc);
    double vx = cxy, vy = lam1 - cxx;
    if (std::hypot(vx,vy) < 1e-9){ vx=1.0; vy=0.0; }
    double nv = std::hypot(vx,vy); vx/=nv; vy/=nv;

    // Orient toward odom yaw
    float yaw_deg = odom_yaw_deg_.load();
    auto angdiff = [](float a, float b){ float d=a-b; while(d>180)d-=360; while(d<-180)d+=360; return d; };
    float v_ang_deg = static_cast<float>(std::atan2(vy, vx) * 180.0 / M_PI);
    if (std::fabs(angdiff(v_ang_deg, yaw_deg)) > 90.0f) { vx=-vx; vy=-vy; v_ang_deg = v_ang_deg + (v_ang_deg>0?-180.f:180.f); }
    psi_corr_deg_.store(wrap_deg_0_360(v_ang_deg));

    // Normal and lateral distances
    double nx = -vy, ny = vx;
    std::vector<float> proj; proj.reserve(pts.size());
    for (auto &p: pts) proj.push_back(static_cast<float>(nx*p[0] + ny*p[1]));
    std::vector<float> left, right; left.reserve(proj.size()); right.reserve(proj.size());
    const float eps = 1e-3f;
    for (float t: proj){ if (t>eps) right.push_back(t); else if (t<-eps) left.push_back(-t); }
    auto trimmed_mean = [this](std::vector<float>& v)->float{
      if (v.empty()) return 0.f;
      std::sort(v.begin(), v.end());
      size_t n=v.size(); size_t k=static_cast<size_t>(trim_ratio_*n);
      if (2*k>=n) k=n/4;
      float s=0.f; size_t c=0;
      for(size_t i=k;i<n-k;++i){ s+=v[i]; ++c; }
      return c? s/static_cast<float>(c) : 0.f;
    };
    float dR = trimmed_mean(right);
    float dL = trimmed_mean(left);
    last_dR_ = dR; last_dL_ = dL;
    float e_perp = 0.5f*(dR - dL);

    // LCP desired angle
    float psi_d = wrap_deg_pm180(v_ang_deg) - static_cast<float>(k_perp_) * static_cast<float>(std::atan(e_perp / std::max(0.05, look_L_)) * 180.0 / M_PI);
    psi_d = wrap_deg_pm180(psi_d);

    // EMA circular
    float pr = psi_d * static_cast<float>(M_PI) / 180.0f;
    float c = std::cos(pr), s = std::sin(pr);
    if (!ema_init_) { ema_c_=c; ema_s_=s; ema_init_=true; }
    else {
      float a = static_cast<float>(ema_alpha_);
      ema_c_ = (1.0f-a)*ema_c_ + a*c;
      ema_s_ = (1.0f-a)*ema_s_ + a*s;
      float n = std::hypot(ema_c_, ema_s_); if (n>1e-9f){ ema_c_/=n; ema_s_/=n; }
    }
    float psi_d_smooth = std::atan2(ema_s_, ema_c_) * 180.0f / static_cast<float>(M_PI);
    if (psi_d_smooth < 0) psi_d_smooth += 360.0f;
    psi_d_deg_.store(psi_d_smooth);

    // Gate laps
    if (gate_init_) {
      float px = odom_x_.load(), py = odom_y_.load();
      float g = bx_*(px - C0x_) + by_*(py - C0y_);
      if ((g>0 && last_g_<=0) || (g<0 && last_g_>=0)) {
        float sp = odom_speed_.load();
        float d = std::fabs(angdiff(odom_yaw_deg_.load(), v_ang_deg));
        auto nowt = now();
        double dt = (nowt - last_lap_stamp_).seconds();
        if (sp > gate_speed_min_ && d < gate_yaw_tol_deg_ && dt > gate_cooldown_s_) {
          laps_++;
          last_lap_stamp_ = nowt;
          if (mode_ == Mode::RUN_LCP && laps_ >= target_laps_) {
            mode_ = Mode::RETURN_TO_ZONE;
            hold_started_ = false;
          }
        }
      }
      last_g_ = g;
    }

    // Return-to-zone steering with guard rails
    if (mode_ == Mode::RETURN_TO_ZONE) {
      float x = odom_x_.load(), y = odom_y_.load();
      float psi_ret = std::atan2(static_cast<float>(C0y_ - y), static_cast<float>(C0x_ - x)) * 180.0f / static_cast<float>(M_PI);
      if (psi_ret < 0) psi_ret += 360.0f;

      float min_wall = std::min(last_dL_, last_dR_);
      float psi_target = (min_wall < guard_dist_) ? psi_d_smooth : psi_ret;

      // push through same EMA (re-use)
      float pr2 = (wrap_deg_pm180(psi_target)) * static_cast<float>(M_PI) / 180.0f;
      float c2 = std::cos(pr2), s2 = std::sin(pr2);
      float a = static_cast<float>(ema_alpha_);
      ema_c_ = (1.0f-a)*ema_c_ + a*c2;
      ema_s_ = (1.0f-a)*ema_s_ + a*s2;
      float n = std::hypot(ema_c_, ema_s_); if (n>1e-9f){ ema_c_/=n; ema_s_/=n; }
      float psi_rt_smooth = std::atan2(ema_s_, ema_c_) * 180.0f / static_cast<float>(M_PI);
      if (psi_rt_smooth < 0) psi_rt_smooth += 360.0f;
      last_angle_out_deg_.store(psi_rt_smooth);
    } else {
      last_angle_out_deg_.store(psi_d_smooth);
    }

    // Publish debug angle
    std_msgs::msg::Float32 dbg; dbg.data = last_angle_out_deg_.load();
    angle_pub_->publish(dbg);
  }

  void on_timer() {
    if (!serial_.is_open()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Serial not open, skipping write...");
      return;
    }

    // State machine: RETURN_TO_ZONE completion
    if (mode_ == Mode::RETURN_TO_ZONE) {
      const float half = static_cast<float>(zone_side_ * 0.5);
      const float margin = static_cast<float>(zone_margin_);
      float x = odom_x_.load(), y = odom_y_.load();
      bool in_zone = (std::fabs(x - static_cast<float>(C0x_)) <= (half - margin)) &&
                     (std::fabs(y - static_cast<float>(C0y_)) <= (half - margin));
      float sp = odom_speed_.load();

      if (in_zone && sp < static_cast<float>(stop_speed_thresh_)) {
        if (!hold_started_) { hold_started_ = true; hold_start_ = now(); }
        else {
          if ((now() - hold_start_).seconds() >= hold_time_) {
            mode_ = Mode::DONE;
          }
        }
      } else {
        hold_started_ = false;
      }
    }

    // Pick angle to send
    float deg = last_angle_out_deg_.load();
    if (!std::isfinite(deg)) deg = odom_yaw_deg_.load();
    deg = wrap_deg_0_360(deg);
    uint16_t ang_tenths = clamp_u16(static_cast<int>(deg * 10.0f + 0.5f));

    // Velocity / kp
    float vel_mps = (mode_ == Mode::RETURN_TO_ZONE) ? static_cast<float>(return_speed_)
                   : (mode_ == Mode::DONE) ? 0.0f
                   : last_vel_mps_.load();
    float kp = last_kp_.load();

    uint16_t vel_mmps = clamp_u16(static_cast<int>(vel_mps * 1000.0f + 0.5f));
    uint16_t kp_milli = clamp_u16(static_cast<int>(kp * 1000.0f + 0.5f));

    auto frame = make_frame_v2(ang_tenths, vel_mmps, kp_milli);
    if (!serial_.write_bytes(frame.data(), frame.size())) {
      RCLCPP_WARN(get_logger(), "Serial write failed: %s", serial_.last_error().c_str());
    }
  }

  // -------- Params --------
  std::string port_;
  int baud_;
  int rate_hz_;

  // LCP
  double k_perp_, look_L_, ema_alpha_, trim_ratio_;
  // Gate
  double gate_speed_min_, gate_yaw_tol_deg_, gate_cooldown_s_;
  int    target_laps_;
  // Zone
  double zone_side_, zone_margin_, return_speed_, hold_time_, stop_speed_thresh_, guard_dist_;

  // -------- Serial --------
  SerialPort serial_;

  // -------- ROS --------
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr control_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // -------- State --------
  std::atomic<float> last_vel_mps_{0.0f};
  std::atomic<float> last_kp_{0.0f};

  std::atomic<float> odom_x_{0.0f}, odom_y_{0.0f}, odom_speed_{0.0f}, odom_yaw_deg_{0.0f};

  std::atomic<float> psi_corr_deg_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> psi_d_deg_{std::numeric_limits<float>::quiet_NaN()};
  std::atomic<float> last_angle_out_deg_{std::numeric_limits<float>::quiet_NaN()};
  float ema_c_{1.0f}, ema_s_{0.0f}; bool ema_init_{false};

  bool gate_init_{false};
  float bx_{1.0f}, by_{0.0f};
  double C0x_{0.0}, C0y_{0.0};
  float last_g_{0.0f};
  int laps_{0};
  rclcpp::Time last_lap_stamp_;

  float last_dL_{0.0f}, last_dR_{0.0f};

  Mode mode_{Mode::RUN_LCP};
  bool hold_started_{false};
  rclcpp::Time hold_start_;
};

// --------------------- main ---------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyCommNode>());
  rclcpp::shutdown();
  return 0;
}
