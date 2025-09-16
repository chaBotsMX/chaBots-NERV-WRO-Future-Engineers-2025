# Software Documentation

## General Description

This is an autonomous robot for the WRO Future Engineers 2025 competition, developed with ROS2 using Python. The robot can navigate autonomously, detect obstacles, and detect colored objects.

## System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Hardware      │    │   ROS2 Nodes    │    │   Algorithms    │
│                 │    │                 │    │                 │
│ • Teensy        │────┤ • teensy_comm   │────┤ • Control       │
│ • OTOS Sensor   │────┤ • otos_reader   │────┤ • Odometry      │
│ • RPLiDAR       │────┤ • rplidar_node  │────┤ • Basic SLAM    │
│ • Pi Camera     │────┤ • vision_node   │────┤ • Vision        │
│ • Motors        │    │                 │    │ • Tracking      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Main Components

### 1. Kinetic Obstacle Detection and Hardware Communication
- **File**: `src/teensy_communication/launch/robot.launch.py`
- **Function**:
- **Features**:
  - Teensy configuration
  - Odometry and sensor management

### 2. Computer Vision
- **File**: `src/vision_node/vision_node/color_detection_node.py`
- **Function**: Green, red, and purple object detection
- **Features**:
  - HSV filtering for specific colors
  - Distance and angle calculation
  - Noise filtering

### 3. Localization System (OTOS)
- **File**: `src/otos_reader/otos_reader/otos_node.py`
- **Function**: Provides precise odometry using OTOS sensor
- **Features**:
  - Software bias correction
  - EMA filtering for smoothing
  - ZUPT detection (Zero Velocity Update)
  - Odometry and TF transforms publishing  - Distance and angle calculation

## Data Flow

```
┌──────────┐     ┌─────────────┐     ┌──────────────┐
│ Sensors  │────▶│ ROS2 Nodes  │────▶│ Control      │
│          │     │             │     │ Algorithms   │
│ • OTOS   │     │ • otos_node │     │              │
│ • LiDAR  │     │ • rplidar   │     │ ┌──────────┐ │
│ • Camera │     │ • vision    │     │ │ Decision │ │
│ • IMU    │     │ • teensy    │     │ │ Making   │ │
└──────────┘     └─────────────┘     │ └──────────┘ │
                                     └──────┬───────┘
                                            │
┌──────────────┐     ┌─────────────┐       │
│   Actuators  │◀────│ Commands    │◀──────┘
│              │     │             │
│ • Motors     │     │ /cmd_vel    │
│ • Servo      │     │ /motor_cmd  │
│              │     │ /servo_cmd  │
└──────────────┘     └─────────────┘
```

## Implemented Algorithms

### 1. Navigation

### 2. Obstacle Avoidance

### 3. Color Detection

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

## Control Implementation

### 1. Navigation Control


## System Configuration

### Sensors and Calibrations
- **OTOS**: Units in meters and degrees
- **LiDAR**: RPLiDAR C1
- **Camera**: 1280x720, RGB888 format
- **Focal Length**: 1131 pixels

### Control Parameters


## Robot States

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  START      │───▶│ NAVIGATION  │───▶│ DETECTION   │
│             │    │             │    │             │
│ • Calibrate │    │ • Follow    │    │ • Identify  │
│ • Reset     │    │   walls     │    │   objects   │
│ • Wait      │    │ • Avoid     │    │ • Calculate │
└─────────────┘    │   obstacles │    │   position  │
                   └─────────────┘    └─────────────┘
                          ▲                   │
                          └───────────────────┘
```

## Main ROS2 Topics

| Topic | Type | Description |
|--------|------|-------------|
| `/scan` | LaserScan | LiDAR data |
| `/odom` | Odometry | OTOS odometry |
| `/camera/image_raw` | Image | Camera image |
| `/cmd_vel` | Twist | Velocity commands |
| `/objects/detection` | Float32MultiArray | Detected objects data |
| `/objects/status` | Float32 | Number of detected objects |


## Execution Commands

```bash
# Launch complete robot
ros2 launch teensy_communication robot.launch.py

# Object detection only
ros2 run vision_node color_detection_node

# OTOS odometry only
ros2 run otos_reader otos_node
```

## File Structure

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

## Monitoring and Debug

- **Foxglove Studio**: Real-time visualization
- **RViz**: Trajectories and laser maps
- **OpenCV Windows**: Camera view with detections
- **ROS2 Logs**: Debug information via console

This robot implements a modular architecture that enables autonomous navigation, obstacle detection, and object recognition, optimized for WRO Future Engineers competitions.
