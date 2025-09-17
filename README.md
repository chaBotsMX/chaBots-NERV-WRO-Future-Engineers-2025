

# ChaBots - WRO Future Engineers 2025

<!--<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/v-photos/resources/ChaBotsLogo.png?raw=true" width="250">-->

## Follow us!
  <!-- Facebook -->
  <a href="https://www.facebook.com/chabotsMX/">
    <img src="https://cdn-icons-png.flaticon.com/512/733/733547.png" width="40" alt="Facebook">
  </a>
  <!-- Instagram (con degradado real) -->
  <a href="https://www.instagram.com/chabotsmx/" target="_blank">
    <img src="https://cdn-icons-png.flaticon.com/512/2111/2111463.png" width="40" alt="Instagram">
  </a>
  <!-- YouTube -->
  <a href="https://www.youtube.com/@chabotsmx1956/videos" target="_blank">
    <img src="https://cdn-icons-png.flaticon.com/512/1384/1384060.png" width="40" alt="YouTube">
  </a>
  <!-- Página Web (icono de internet) -->
  <a href="https://www.chabots.mx" target="_blank">
    <img src="https://cdn-icons-png.flaticon.com/512/841/841364.png" width="40" alt="Website">
  </a>

This repository contains the documentation for **ChaBots** participation in the **WRO Future Engineers 2025** category. Our robot was designed and built by a Mexican students team,  passionate about robotics and education.


## 📜 Table of Contents

1. 🧑‍💻 [The Team](#the-team)
2. 🎯 [The Challenge](#the-challenge)
3. 🤖 [Robot Overview](#robot-overview)
4. 🔋 [Sense Overview](#sense-overview)
5. ⚙️ [Mobility Management](#mobility-management)
6. 💻 [Code Overview](#code-overview)
7. 🚧 [Obstacle Management](#obstacle-management)
8. 🛠️ [Construction Guide](#construction-guide)
9. 💰 [Cost Report](#cost-report)
- 📚 [Resources](#resources)
- ©️ [License](#license)
---

## 1. The Team <a name="the-team"></a>
<div align="center">
<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/photos/t-photos/t-photo.png?raw=true">
</div>
<div align="center">
    <h2 style="color:#1e90ff; font-size:2.2em; margin-top:0.5em; margin-bottom:0.2em;">
        <span style="color:#222; background:linear-gradient(90deg,#1e90ff,#00c3ff,#00ffb3,#1e90ff);-webkit-background-clip:text;-webkit-text-fill-color:transparent;">We are <b>ChaBots NERV</b></span> -[:]
    </h2>
</div>

### Roy Iván Barrón Martínez
**Age:** 20\
**Role:** Captain, Electronics & Software Designer

I am a self-taught robotics enthusiast with experience in embedded systems, software, and mechanical integration. my team ChaBots Ocelot won Mexico Robocup soccer Open second place and achieved multiple national awards in programming and robotics.

> "I enjoy setting nearly impossible goals to push myself while learning. I believe that learning should always lead to building something real."

---

### Leonardo Villegas Lopez
**Age:** 20\
**Role:** Mechanical Designer

I am a Mechatronics Engineering student passionate about technology and innovation. I have been a contestant for eight years, winning various regional and national competitions, and participating internationally.
> "I will take any opportunity to grow"

---

### Hiram Jalil Castillo Gutierrez
**Age:** 22\
**Role:** Software Developer

I am a Software Engineer student and I love robotics and programming. I have been a contestant for 8 years, from regional to national competitions, I have participated in 4 Mexico Robocup Soccer and 1 MakeX Robotics Competition, top 3(2019) and runner-up(2019).

> "Anyway, robotics is my passion and I will never forget it."

---

### Diego Vitales Medellín
**Age:** 22\
**Role:** Coach

I've been involved in robotics for 14+ years being a programmer for most of the projects I've taken part in. I've had may regional, national and international experiences. Now I'm working in sharing my knowledge with more people to push further their level and potential as well as helping them achieve their goals and find their passion.

> "I like to face challenges and even more so when it's with more people. Learning and creating something is better when shared."

---

## 2. The Challenge <a name="the-challenge"></a>

The **WRO Future Engineers** challenge pushes students to create fully autonomous self-driving vehicles. Each robot must:

- Navigate a dynamically randomized track
- Detect and avoid colored obstacles (green/red blocks)
- Execute a parallel parking maneuver

Scoring is based on:
- Performance on track
- Obstacle handling
- Documentation quality
- Innovation and engineering rigor

For more indo visit: [WRO Official Site](https://wro-association.org/)

---

## 3. Robot Overview <a name="robot-overview"></a>

 **Name:** Eva

| Front | Back |
|-------|------|
<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/v-photos/national/v-front.jpeg?raw=true" width="250">| <img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/v-photos/national/v-back.jpeg?raw=true" width="250">|

| Left | Right |
|------|-------|
<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/v-photos/national/v-left.jpeg?raw=true" width="250">| <img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/v-photos/national/v-right.jpeg?raw=true" width="250">|

| Top | Bottom |
|------|--------|
<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/v-photos/national/v-top.jpeg?raw=true" width="250">| <img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/v-photos/national/v-bottom.jpeg?raw=true" width="190">

---

## 4. Sense Overview <a name="sense-overview"></a>

### 4.1. RPLiDAR C1
360° laser scanner for environmental mapping and obstacle detection.

**Tech specs:**
- 360° scanning with 0.9° resolution
- Up to 8m range with 10Hz update rate
- Quality filtering for reliable data
- ROS2 integration via `rplidar_ros` package

**Link:** [RPLiDAR C1](https://www.slamtec.com/en/C1)

### 4.2. Raspberry Pi Camera V3
High-resolution camera for color object detection.

**Tech specs:**
- 12MP IMX708 Quad Bayer sensor and features a High Dynamic Range mode
- Supports 1080p30, 720p60, and VGA90 video modes

**Link:** [Raspberry Pi Camera V3](https://www.raspberrypi.com/products/camera-module-3/)

### 4.3. SparkFun Optical Tracking Odometry Sensor
High-precision odometry sensor for accurate position tracking.

**Tech specs:**
- Measures linear and angular displacement
- High-resolution optical flow sensor
- ROS2 integration via custom `otos_reader` node

**Link:** [SparkFun OTOS](https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html)

## 5. Mobility Management <a name="mobility-management"></a>

### 5.1. Gearbox:
<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/models/gearbox/gearbox-assemble.png?raw=true.png">

**Motor Model:** Maxon DCX19
The team opted for custom steel shafts with a 4 mm diameter, which were connected to a Maxon DCX19 motor. This motor provided sufficient power and torque to meet the project's performance requirements, with a maximum speed of 600 RPM. The gear ratio between the motor and the output gear is 1:1, as higher speeds were not required at the time.
The transmission base and gears were designed and manufactured by the team. To maximize strength and durability, Polymaker PTG CF filament was used on a Creality K2 Plus Combo printer. To improve efficiency and reduce mechanical wear, double helical gears were designed and printed.
The steel shafts were manually sized from steel rod. These were cut to the required length and, using a Dremel tool, shaped into a D-shape to ensure a firm grip with the wheel hubs. The rear wheels were also custom-made with a 3D-printed D-shaped axle for secure fit.


### 5.2. Steering System
<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/models/steering-system/steering-system-assembly.png?raw=true">

**Servo Model:** HS 85mg
For the steering system, the goal was to make the mechanism as simple as possible, as this would allow for quick and easy manufacturing. It was decided to mount the HS 85mg servo on a 3D-designed and printed base, just like all the other components. The servo is connected to the beam that connects to the wheel mounts. These mounts are made to fit a LEGO axle without slipping.
For the front wheels, the team used LEGO Spike rubber tires after determining that manufacturing them in-house was not feasible.

### 5.3. Chasis
<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/docs-nacional/models/v-assembly.png?raw=true">
The chassis is the robot's main structure, as all other systems are mounted on it. A modular design was chosen to facilitate assembly and maintenance. The chassis is made of carbon fiber, which was cut in China.
The steering system is mounted on the chassis using 20mm-high M3 posts. The odometer PCB is anchored below the steering system, as this makes better use of space. The gearbox is mounted directly to the rear of the chassis, and the Raspberry Pi 5 is mounted on it using 20mm-high M2.5 posts. The main PCB is mounted in the middle, and the Lidar base is mounted on 20mm-high M3 posts. The Raspberry Pi camera v2 base is mounted on the Lidar base using 40mm-high M3 posts.
Using these poles helped us keep the robot as low as possible, allowing the Lidar sensor to be level with the runway walls.

---


## 6. Code Overview <a name="code-overview"></a>

This is an autonomous robot developed with ROS2 using Python. The robot can navigate autonomously, detect obstacles, and detect colored objects.

### 6.1. System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Hardware      │    │   ROS2 Nodes    │    │   Algorithms    │
│                 │    │                 │    │                 │
│ • Teensy        │────┤ • teensy_comm   │────┤ • Control       │
│ • OTOS Sensor   │────┤ • otos_reader   │────┤ • Odometry      │
│ • RPLiDAR       │────┤ • rplidar_node  │────┤ • Track Map     │
│ • Pi Camera     │────┤ • vision_node   │────┤ • Vision        │
│ • Motors        │    │                 │    │ • Tracking      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 6.2. Implementations

#### 6.2.1. Kinetic Obstacle Detection and Hardware Communication
- **File**: `src/teensy_communication/launch/robot.launch.py`
- **Function**:
- **Features**:
  - Teensy configuration
  - Odometry and sensor management

#### 6.2.2. Computer Vision
- **File**: `src/vision_node/vision_node/color_detection_node.py`
- **Function**: Green, red, and purple object detection
- **Features**:
  - HSV filtering for specific colors
  - Distance and angle calculation
  - Noise filtering

#### 6.2.3. Localization System (OTOS)
- **File**: `src/otos_reader/otos_reader/otos_node.py`
- **Function**: Provides precise odometry using OTOS sensor
- **Features**:
  - Software bias correction
  - EMA filtering for smoothing
  - ZUPT detection (Zero Velocity Update)
  - Odometry and TF transforms publishing  - Distance and angle calculation

### 6.3. Data Flow

```
┌──────────┐     ┌─────────────┐     ┌──────────────┐
│ Sensors  │────▶│ ROS2 Nodes  │────▶│ Control     │
│          │     │             │     │ Algorithms   │
│ • OTOS   │     │ • otos_node │     │              │
│ • LiDAR  │     │ • rplidar   │     │ ┌──────────┐ │
│ • Camera │     │ • vision    │     │ │ Decision │ │
│ • IMU    │     │ • teensy    │     │ │ Making   │ │
└──────────┘     └─────────────┘     │ └──────────┘ │
                                     └──────┬───────┘
                                            │
┌──────────────┐     ┌─────────────┐       	│
│   Actuators  │◀────│ Commands    │◀──────┘
│              │     │             │
│ • Motors     │     │ /cmd_vel    │
│ • Servo      │     │ /motor_cmd  │
│              │     │ /servo_cmd  │
└──────────────┘     └─────────────┘
```

### 6.4. Implemented Algorithms

#### 6.4.1. Navigation
Using RPLIDAR C1 Ros2 package we can read the topic from our own node,
```c++
void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const float angle_min = msg->angle_min;
    const float angle_inc = msg->angle_increment;
    const float pi = static_cast<float>(M_PI);
    lidarMSG.clear();
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        const float ang = angle_min + angle_inc * static_cast<float>(i);
        if (ang < -0.5235f && ang > -2.6180f || ang > pi)
            continue; // 0..180°
        const float r = msg->ranges[i];
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;
        lidarMSG.push_back({ang, pointAngX(ang, r), pointAngY(ang, r), r});
    }
}
```
This call back is called only when a the lidar node post a new message in topic, this piece only storage a cloud of points send by the lidar, the next step is procces this information to get the actual position of the robot in the track, we use the OTOS to update this information every call, because of the lidar only posting at 10hz is to slow, it means it only have new information every 100ms that make imposible to have a smoth move, the OTOS post at 100Hz, meaning we can smoth our trajectory 10 times more using this, we dont use absolute infromation like coordinates in x or y axis, we take the changue in cm betwen the new read and the last read and add it to the cloud of points, also we calculate some infromation to help our robot to not colide with the walls, like the distance in front, left and rigth, this help to calculate optimal PD controller values relative to robots position and speed.
```c++

void getOffsetsFromLidar()
{
    if (new_otos_data.load())
    {
        const float yaw_prev = deg2rad(lastYaw.load());
        const float dx_w = posX_.load() - lastPosX.load();
        const float dy_w = posY_.load() - lastPosY.load();
        const float dth = wrapPi(deg2rad(yaw.load() - lastYaw.load()));
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
            spt.angle = wrapPi(std::atan2(yr, xr));
            spt.mag = std::hypot(xr, yr);
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
    for (const auto &s : lidarMSG)
    {
        const float ang = s.angle;
        const float ang_eff = ang + phi; // MISMA rotación para todos
        sumX += s.x;
        sumY += s.y;
        if (ang_eff >= 0.0f && ang_eff < 0.78f)
        { // izquierda ~ 0..45°
            leftDis += s.mag * std::cos(ang_eff - 0.0f);
            ++sum_left;
        }
        if (ang_eff > 2.35f && ang_eff <= static_cast<float>(M_PI))
        { // derecha ~ 135..180°
            rightDis += s.mag * std::cos(ang_eff - static_cast<float>(M_PI));

            ++sum_right;
        }
        if (ang_eff >= 1.39f && ang_eff <= 1.74f)
        { // frente ~ 80..100°
            frontDis += s.mag * std::cos(ang_eff - M_PI_2);
            ++sum_front;
        }
    }
    absolute_angle.store(rad2deg(std::atan2(sumY, sumX)));
    leftDis /= sum_left;
    rightDis /= sum_right;
    frontDis /= sum_front;
    totalDis = std::fabs(leftDis) + std::fabs(rightDis);
    anchoCorredor.store(totalDis);
    setpoint = totalDis / 2.0f;
    frontWallDistance.store(frontDis);
    centeringOffset.store(setpoint - std::fabs(rightDis));
}

```

Talking about optimal values, we have a simple track map function to be able to reach max speed in every type of turn, becasue the track isnt symetric, it can struggle with the same valiue if the wall is closer, fo example at 60 cm instead of the standar 100 cm, this function also let us now in what turn it is and know when to stop and to get the drive direction too.

```c++
void getActualSector()
{
    float orientation = heading360.load();
    int thisSector = actualSector.load();
    int thisSectorUpperLimit = sectoresAngs[0][thisSector];
    int thisSectorLowerLimit = sectoresAngs[1][thisSector];

    if (thisSector == 0)
    {
        orientation >= 180 ? orientation -= 360 : orientation = orientation;
        if (orientation < -75)
        {
            actualSector.store(3);
            inTurn.store(false);
            if (driveDirection.load() == 0)
            {
                driveDirection.store(2);
            }
        }
        else if (orientation > 75)
        {
            actualSector.store(1);
            inTurn.store(false);
            if (driveDirection.load() == 0)
            {
                driveDirection.store(1);
            }
        }
    }
    else if (static_cast<int>(orientation) > thisSectorUpperLimit + 30)
    {
        thisSector++;
        inTurn.store(false);
        thisSector > 3 ? thisSector = 0 : thisSector = thisSector;
        actualSector.store(thisSector);
    }
    else if (static_cast<int>(orientation) < thisSectorLowerLimit - 30)
    {
        thisSector--;
        inTurn.store(false);
        thisSector < 0 ? thisSector = 3 : thisSector = thisSector;
        actualSector.store(thisSector);
    }
}

```
#### 6.4.2. Obstacle Avoidance

The obstacle avoidance system (`teensy_obs_node.cpp`) implements a multi-sensor approach for autonomous navigation in environments with colored obstacles. This system integrates LIDAR, odometry, and computer vision to provide intelligent obstacle detection and avoidance capabilities.

#### Overview

The obstacle avoidance node operates as a state machine with three primary operational modes:
- **Normal Navigation**: Wall-following behavior with sector-based movement
- **Obstacle Avoidance**: Color-based object detection and avoidance maneuvers
- **Turn Execution**: Multi-step turning algorithms for corner navigation

##### Vision Integration
```cpp
// Vision system subscribers
object_distance_sub_ = create_subscription<std_msgs::msg::Float32>("/object/distance", 10, ...);
object_angle_sub_ = create_subscription<std_msgs::msg::Float32>("/object/angle", 10, ...);
object_color_sub_ = create_subscription<std_msgs::msg::Float32>("/object/color", 10, ...);
object_status_sub_ = create_subscription<std_msgs::msg::Float32>("/object/status", 10, ...);
```

The system subscribes to vision node outputs providing:
- **Distance**: Object distance in centimeters
- **Angle**: Relative angle to object in degrees
- **Color**: Object classification (0 = green, 1 = red)
- **Status**: Detection status (0 = no object, 1 = object detected)

#### Sensor Processing
The node processes LIDAR data to extract directional distance measurements:
```cpp
// LIDAR sector analysis
if(a < -1.3962f && a > -1.7453f) { sumBack += r; ++totalBack; }      // Back sector
if (a > 1.39 && a < 1.7453f) { sumFront += r; ++totalFront; }        // Front sector
if( a > 0.0f && a < 0.5235f) { sumLeft += r; ++totalLeft; }          // Left sector
if( a > 2.79252f && a < 3.141592f) { sumRight += r; ++totalRight; }  // Right sector
```

### Obstacle Avoidance Algorithm
#### Color-Based Avoidance Strategy

**Green Obstacle Avoidance:**
```cpp
if(color == 0){ // Green obstacle
    angle = angle - 20;  // Bias left by 20 degrees
    if(angle < 0){
        float returnANG = 90 + angle;
        // Steer away from green obstacle
        mover(returnANG, 50, 0);
    }
}
```

**Red Obstacle Avoidance:**
```cpp
else if(color == 1){ // Red obstacle
    angle = angle + 20;  // Bias right by 20 degrees
    if(angle > 0){
        float returnANG = 90 + angle;
        // Steer away from red obstacle
        mover(returnANG, 50, 0);
    }
}
```

#### Decision Making Logic
The main control loop implements a hierarchical decision structure:

```cpp
void on_timer() {
    getActualSector();  // Update sector tracking
    int isObs = object_status_.load();

    if(inturn.load()){
        rutinaGirar();  // Execute turn routine
        return;
    }
    else if(isObs == 1){
        // Object detected - execute avoidance
        executeObstacleAvoidance();
    }
    else{
        // Normal navigation
        orientar();  // Maintain heading
        checkTurnConditions();
    }
}
```
#### Adaptive Turning System
The system implements three distinct turning algorithms based on available space:
##### Turn Type Classification
```cpp
if(outWallDistance >= 0.8f){
    turntype_.store(3);  // Wide turn - standard maneuver
}
else if(outWallDistance < 0.8f && outWallDistance >= 0.3f){
    turntype_.store(2);  // Medium turn - complex multi-step
}
else if(outWallDistance < 0.3f){
    turntype_.store(1);  // Close turn - backup required
}
```

##### Turn Execution State Machine

**Type 1 - Close Turn (Backup Required):**
1. **Step 0**: Align to target heading
2. **Step 1**: Reverse until sufficient clearance (`distBack < 0.5m`)

**Type 2 - Medium Turn (Complex Maneuver):**
1. **Step 0**: Approach corner with 45° offset
2. **Step 1**: Reverse turn with heading correction
3. **Step 2**: Complete turn in reverse

**Type 3 - Wide Turn (Standard):**
1. **Step 0**: Forward approach until close to wall
2. **Step 1**: Execute directional turn (150° left / 30° right)
3. **Step 2**: Complete turn in reverse

#### Performance Characteristics

##### Real-Time Operation
- **Control Frequency**: 100 Hz (10ms cycle time)
- **Serial Communication**: 2 Mbps UART with checksum validation
- **Thread Safety**: Atomic variables for all shared state

##### Sensor Integration Timing
- **LIDAR Processing**: Real-time sector analysis
- **Vision Integration**: Object detection at 30 FPS
- **Odometry Fusion**: 100 Hz pose updates

#### Navigation Parameters
```cpp
// Fixed speed for obstacle round
const int OBSTACLE_SPEED = 50;  // PWM value

// Distance thresholds
const float TURN_THRESHOLD = 1.0f;      // Front distance to initiate turn
const float ALIGNMENT_TOLERANCE = 5.0f; // Heading error tolerance (degrees)
const float CLOSE_DISTANCE = 0.3f;      // Close turn threshold
const float WIDE_DISTANCE = 0.8f;       // Wide turn threshold
```

### Safety Systems

##### Collision Prevention
- Continuous front distance monitoring
- Emergency stop capability
- Multi-sensor validation before maneuvers

##### State Validation
- Finite state machine prevents invalid transitions
- Atomic operations ensure thread-safe state updates
- Timeout mechanisms for stuck conditions

##### Error Recovery
- Automatic retry for failed turn sequences
- Fallback to normal navigation if vision system fails
- Robust serial communication with error detection

##### Vision-LIDAR Fusion

The system combines vision and LIDAR data for enhanced obstacle detection:

1. **Vision System**: Provides precise object classification and angular position
2. **LIDAR System**: Validates distances and provides environmental context
3. **Fusion Logic**: Cross-validates detections and selects optimal avoidance strategy

This multi-modal approach ensures robust performance in complex environments with varying lighting conditions and obstacle configurations.

##### Integration with WRO Future Engineers Challenge

The implementation aligns with the WRO Future Engineers challenge requirements by enabling autonomous navigation, obstacle detection, and avoidance in a dynamic environment. The system's modular design allows for easy adaptation to different track layouts and obstacle placements, ensuring compliance with competition rules.

#### 6.4.3. Color Detection

Using OpenCV to detect green, red, and purple objects in the camera feed. The algorithm filters colors in HSV space, finds contours, and calculates distance and angle based on object size and position.

```python
# HSV filtering for green
mask_green = cv2.inRange(hsv, lower_green, upper_green)

# HSV filtering for red (two ranges)
mask_r1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask_red = cv2.bitwise_or(mask_r1, mask_r2)

# HSV filtering for purple
mask_purple = cv2.inRange(hsv, lower_purple, upper_purple)

# Noise filtering
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
mask_purple = cv2.morphologyEx(mask_purple, cv2.MORPH_OPEN, kernel)

# Distance calculation
distance = (KNOWN_WIDTH * FOCAL_LENGTH) / bounding_box_width
```

### 6.5. Control Implementation
Using the sum of every point from the lidar we get a vector that tell us where to go, this for it self isnt optimal cause it tell us where is more space, so we add a vector pointing to the IMU target, this help us to get more smooth and optimal trajectories more than just drive where you can.
```c++
float angleProccesing(float kpNoLinear = 0.75f, float maxOut = 30.0f, bool yawMode = false, float yawKp = 0.025f)
{
    float yawHelpError = wrap_deg180(heading360.load() - sectoresAbsAng[actualSector.load()]) * yawKp;
    if (inTurn.load())
    {
        if (driveDirection.load() == 1)
        { // horario
            yawHelpError = wrap_deg180(heading360.load() - wrap_360(sectoresAbsAng[actualSector.load()] - 90.0f)) * 0.5;
        }
        else if (driveDirection.load() == 2)
        { // antihorario
            yawHelpError = wrap_deg180(heading360.load() - wrap_360(sectoresAbsAng[actualSector.load()] + 90.0f)) * 0.5;
        }
    }
    float angleInput = absolute_angle.load();

    yawMode ? angleInput = angleInput + yawHelpError : angleInput = angleInput;

    float angularError = 90.0f - angleInput;
    float beta = kpNoLinear / maxOut;

    return maxOut * std::tanh(angularError / (maxOut / kpNoLinear));
}


```
With this info now is time to make the robot move to that direction, its not enough with send a simple pwm, because speed have to changue in turns and in corridors, and also is not the same if is in a wide corridor and the next also is, or if its in a wide corridor and the next is narrow, so to go to the max speed we can we use a ACDC controller, this make the robot have the ability to go not only at a constant speed measured in m/s, this make the robot able to activly brake if is needed.
```c++
int controlACDA(float targetSpeed)
{
    float pwm = 0, jerk = 10;
    float error = targetSpeed - speed.load();
    float aproxPwm = 35.0f;

    if (targetSpeed < 0.6f)
    {
        aproxPwm = 35.0f;
    }
    else if (targetSpeed < 1.2f)
    {
        aproxPwm = 40.0f;
    }
    else
    {
        aproxPwm = 60.0f;
    }

    float lastPwmLocal = lastPwm.load();
    float kp = 8.25f; // Valor a determinar
    float kd = 0.1f;  // Valor a determinar

    pwm = (error * kp) + ((error - lastError.load()) / 0.01) * kd;
    pwm = clampf(clampf(pwm + aproxPwm, lastPwmLocal - jerk, lastPwmLocal + jerk), 0, 255);
    lastPwm.store(pwm);
    lastError.store(error);

    if (error < -0.5f || targetSpeed == 0)
        return 0;

    if (error < -0.1f)
        return 1;

    return static_cast<int>(pwm);
}

```
This helps a lot but is not the only controller we have, cause the speed isnt the same if is correctly aligned or if is near to collide with the walls.

```c++
float objectiveAngleVelPD(float vel_min, float vel_max)
{
    const float alpha = 0.3f; // suavizado EMA
    const float dt = 0.01f;   // 10 ms (timer)
    float a = absolute_angle.load();

    if (!std::isfinite(a))
        return vel_min; // sin reducción cuando no hay ángulo

    // Error envuelto a [-180, 180]
    float e = 90.0f - a;

    while (e > 180.0f)
        e -= 360.0f;

    while (e < -180.0f)
        e += 360.0f;

    // Derivada cruda con el error previo

    float e_prev = lastVelErr.load();
    float raw_derivada = (e - e_prev) / dt; // deg/s "amplificado"

    lastVelErr.store(e);

    // EMA correcto: y(k) = y(k-1) + alpha * (x(k) - y(k-1))

    float der_prev = de_f.load();
    float derivada = der_prev + alpha * (raw_derivada - der_prev);

    de_f.store(derivada);

    const float kp = 0.04f;              // m/s por grado
    const float kd = 0.005f;             // m/s por (grado/seg filtrado)

    float reduccion = kp * std::fabs(e); /*+ kd * std::fabs(derivada);*/

    return clampf(reduccion, vel_min, vel_max); // p.ej. [0.0f, 0.8f]
}
```
#### 6.5.1. Navigation Control
For navigation we first map the track, the first lap is made at a lowe speed and we save dara, after this we calculate optimal speed and PD values in every section of the track.
```c++
void getOptimalValues()
{
    float actualSize = getCurrentSectorSize();
    float nextSize = getNextSectorSize();

    if (actualSize >= 0.80f)
    {
        if (nextSize >= 0.80f)
        {
            optimalSpeed.store(5.0f);
            optimalKp.store(0.20f);
            optimalSpeedTurn.store(4.0f);
            optimalKpTurn.store(0.30f);
            yawMult.store(0.125f);
            turnYawMult.store(0.1f);
            turnDis.store(1.2f);
        }

        else if (nextSize < 0.80f)
        {
            optimalSpeed.store(1.0f);
            optimalKp.store(0.50f);
            optimalSpeedTurn.store(1.8f);
            optimalKpTurn.store(0.60f);
            yawMult.store(0.125f);
            turnYawMult.store(0.125f);
            turnDis.store(0.7f);
        }
    }

    else if (actualSize < 0.80f)
    {
        if (nextSize >= 0.80f)
        {
            optimalSpeed.store(1.7f);
            optimalKp.store(0.50f);
            optimalSpeedTurn.store(1.7f);
            optimalKpTurn.store(0.75f);
            yawMult.store(0.125f);
            turnYawMult.store(0.05f);
            turnDis.store(1.0f);
        }

        else if (nextSize < 0.80f)
        {
            optimalSpeed.store(3.0f);
            optimalKp.store(0.50f);
            optimalSpeedTurn.store(1.5f);
            optimalKpTurn.store(0.60f);
            yawMult.store(0.125f);
            turnYawMult.store(0.125f);
            turnDis.store(0.8f);
        }
    }
}
```
### 6.6. System Configuration

#### 6.6.1. Sensors and Calibrations
- **OTOS**: Units in meters and degrees
- **LiDAR**: RPLiDAR C1
- **Camera**: 1280x720, RGB888 format
- **Focal Length**: 1131 pixels

#### 6.6.2. Control Parameters


### 6.7. Robot States

```
┌─────────────┐    ┌─────────────┐     ┌─────────────┐
│  START      │───>│ NAVIGATION  │───> │ DETECTION   │
│             │    │             │     │             │
│ • Calibrate │    │ • Simple    │     │ • Identify  │
│ • Reset     │    │   SLAM      │     │   objects   │
│ • Wait      │    │ • Avoid     │     │ • Calculate │
└─────────────┘    │   obstacles │     │   position  │
                   └─────────────┘     └─────────────┘
                          ^                    |
                          └────────────────────┘
```

### 6.8. ROS2

#### 6.8.1. Topics
| Topic | Type | Description |
|--------|------|-------------|
| `/scan` | LaserScan | LiDAR data |
| `/odom` | Odometry | OTOS odometry |
| `/camera/image_raw` | Image | Camera image |
| `/cmd_vel` | Twist | Velocity commands |
| `/objects/detection` | Float32MultiArray | Detected objects data |
| `/objects/status` | Float32 | Number of detected objects |

#### 6.8.2. ROS2 Diagram

<img src="https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025/blob/sw-docs/software-diagrams/ros2-diagram.png?raw=true">

### 6.9. Execution Commands

```bash
# Launch complete robot
ros2 launch teensy_communication robot.launch.py

# Object detection only
ros2 run vision_node color_detection_node

# OTOS odometry only
ros2 run otos_reader otos_node
```

### 6.10. File Structure

```
src/
├── vision_node/           # Color object detection
│   └── color_detection_node.py
├── otos_reader/          # OTOS odometry
│   └── otos_node.py
└── teensy_communication/ # General coordination
    └── launch/
        └── robot.launch.py
    └── src/
        ├── teensy_comm_node.cpp
        ├── teensy_obs_node.cpp

```

### 6.11. Monitoring and Debug

- **Foxglove Studio**: Real-time visualization
- **RViz**: Trajectories and laser maps
- **OpenCV Windows**: Camera view with detections
- **ROS2 Logs**: Debug information via console

---

## 7. Obstacle Management <a name="obstacle-management"></a>

The robot detects and reacts to obstacles in real-time using multiple sensor modalities:

### 7.1. Detection Methods
- **Primary:** Enhanced color detection via PiCamera2 system
- **Verification:** LIDAR distance measurements for obstacle confirmation and navigation
- **Backup:** OTOS position tracking for navigation consistency

### 7.2. Response Algorithms
- **Dynamic turning decision system** based on cube color and position
- **Follow-the-object mode** with PID steering based on cube centroid
- **Multi-sensor verification** to reduce false positives
- **Adaptive speed control** based on obstacle proximity

---

## 8. Construction Guide <a name="construction-guide"></a>

**Models file folder:** `models/`

### 8.1. Steps
- Step 1: 3D designing
- Step 2: 3D printing
- Step 3: Electronic layout
- Step 4: Wiring
- Step 5: Mounting
- Step 6: Programming
- Step 7: Testing

### 8.2. Construction Tools
- 3D Printer (Creality K2 Plus, QIDI Q1 Pro)
- Polymaker PTG CF filament
- Mini Electric Soldering Iron Kit TS101
- Dremel Tool
- Screwdriver Set Fanttik


## 9. Cost Report <a name="cost-report"></a>

| Item                         | Qty | Unit Cost (MXN) | Total (MXN) |
|------------------------------|-----|------------------|-------------|
| Teensy 4.0                   | 1   | $800              | 800         |
| Raspberry Pi 5                | 1   | $2,800             | $2,800        |
| RPlidar C1                    | 1   | $2,500             | $2,500        |
| Raspberry Pi Camera 12mp V3   | 1   | $920              | $920         |
| Raspberry Pi 5 Camera Cable   | 1   | $64               | $64          |
| 2.2Ah LiPo 11.1V Battery     | 1   | $600              | $600         |
| 1Ah LiPo 3.3V Battery     | 1   | $70               | $70          |
| Maxon Motor DCX19            | 1   | $8,500             | $8,500        |
| HS85MG Micro Servo            | 1   | $2,000             | $2,000        |
| SparkFun OTOS                 | 1   | $2,400             | $2,400        |
| POLYMAKER PLA Filament (prototypes)    | 1kg   | $900        | $900         |
| POLYMAKER PLA-CF Filament (finals)     | 0.5kg   | $450       | $450         |
| Carbon Fiber                  | 1   | $2,000             | $2,000        |
| SMD Components & Misc.   | -   |         $1,500      | $1,500        |
| PCB Manufacturing             | 1   | $800              | $800         |
| Spike Wheels (LEGO)         | 4   | $150              | $600         |
| EV3 Wheels (LEGO)          | 2   | $10              | $20         |
| **Total**                     |     |                  | **$26,924**|


---

## Resources <a name="resources"></a>

- [Chabots Main Site](https://www.chabots.mx)
- [WRO Future Engineers Rules PDF](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
- [GitHub Repos](https://github.com/chaBotsMX/chaBots-NERV-WRO-Future-Engineers-2025)

---

## License <a name="license"></a>

```
MIT License
Permission is hereby granted, free of charge, to any person obtaining a copy of this software.
```

---

> *Document maintained by Chabots | Last updated: Sept 2025*

<!--stackedit_data:
eyJoaXN0b3J5IjpbMTcyMzM3ODYxNCwtMzc2NTM2MDM5LDM1ND
c4NDQyMCwxMjQ4Mzg0MTM1LC0yODM3NTcxNywtMTMyNzEwNTIy
MywxMjg3Nzk2NjQsLTQ4MTYzMzM4MF19
-->


> Written with [StackEdit](https://stackedit.io/).
