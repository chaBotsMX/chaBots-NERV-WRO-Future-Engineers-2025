#include "TeensyCommNode.h"

TeensyCommNode::TeensyCommNode() : Node("teensy_comm") {
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

  // Suscripción al tópico de odometría
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&TeensyCommNode::on_odom, this, std::placeholders::_1)
  );

  timer_ = create_wall_timer(10ms, std::bind(&TeensyCommNode::on_timer, this));
}

static inline float TeensyCommNode::pointAngFromRobot(float iteration, float angleInc, float minAngle){
  return  minAngle + angleInc * iteration; // rad
}

static inline float TeensyCommNode::getDiffAngle(float ang1, float r1, float ang2, float r2){
  const float dx = pointAngX(ang2, r2) - pointAngX(ang1, r1);
  const float dy = pointAngY(ang2, r2) - pointAngY(ang1, r1);
  return std::atan2(dy, dx); // rad
}

static inline float TeensyCommNode::getEuclideanDistance(float ang1, float r1, float ang2, float r2){
  const float dx = pointAngX(ang2, r2) - pointAngX(ang1, r1);
  const float dy = pointAngY(ang2, r2) - pointAngY(ang1, r1);
  return std::hypot(dx, dy);
}

static float TeensyCommNode::angulo_positivo(float a) {
  const float two_pi = 2.0f * static_cast<float>(M_PI);
  while (a >= two_pi) a -= two_pi;
  while (a < 0.0f)    a += two_pi;
  return a;
}

// ---------- Empaquetado (6 bytes) ----------
// [0xAB][ANG_H][ANG_L][PWM][KP][CHK]
static std::array<uint8_t, 6> TeensyCommNode::empaquetar(uint16_t ang_tenths, uint8_t pwm_byte, uint8_t kp_byte) {
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
void TeensyCommNode::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  const float angle_min = msg->angle_min;
  const float angle_inc = msg->angle_increment;
  const float pi        = static_cast<float>(M_PI);

  // (A) Ángulo global (vectorial) para publicar como Float32
  double sum_x = 0.0, sum_y = 0.0;
  size_t used = 0;

  //vector con mas espacio absoluto
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
          

  
/*
  float angle_deg = std::numeric_limits<float>::quiet_NaN();

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

  cluster_pub_->publish(arr);*/
}

void TeensyCommNode::on_timer() {
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

void TeensyCommNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extraer yaw (heading) del quaternion de la odometría
  const auto& q = msg->pose.pose.orientation;
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  float yaw = std::atan2(siny_cosp, cosy_cosp); // radianes
  float yaw_deg = yaw * 180.0f / static_cast<float>(M_PI);
  RCLCPP_INFO(this->get_logger(), "Heading: %.2f", yaw_deg);
  heading.store(yaw_deg);
}
