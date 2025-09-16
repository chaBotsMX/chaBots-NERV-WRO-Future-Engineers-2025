
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
  <a href="https://www.chabotsmx.com" target="_blank">
    <img src="https://cdn-icons-png.flaticon.com/512/841/841364.png" width="40" alt="Website">
  </a>

This repository contains the documentation for **ChaBots** participation in the **WRO Future Engineers 2025** category. Our robot was designed and built by a Mexican students team,  passionate about robotics and education.

## Table of Contents

- [The Team](#the-team)
- [The Challenge](#the-challenge)
- [Robot Overview](#robot-overview)
- [Mobility Management](#mobility-management)
- [Power and Sense Management](#power-and-sense-management)
- [Code Overview](#code-overview)
- [Obstacle Management](#obstacle-management)
- [Construction Guide](#construction-guide)
- [Cost Report](#cost-report)
- [Resources](#resources)
- [License](#license)

---

## The Team <a name="the-team"></a>

### Hiram Jalil Castillo Gutierrez
**Age:** 22\
**Role:** Software Developer

I am a Software Engineer student and I love robotics and programming. I have been a contestant for 8 years, from regional to national competitions, I have participated in 4 Mexico Robocup Soccer and 1 MakeX Robotics Competition, top 3(2019) and runner-up(2019).

> "Anyway, robotics is my passion and I will never forget it."

---

### Leonardo Villegas Lopez
**Age:** 20\
**Role:** Mechanical Designer

I am a Mechatronics Engineering student passionate about technology and innovation. I have been a contestant for eight years, winning various regional and national competitions, and participating internationally.


---
### Roy Iván Barrón Martínez
**Age:** 20\
**Role:** Captain, Electronics & Software Designer

I am a self-taught robotics enthusiast with experience in embedded systems, software, and mechanical integration. my team ChaBots Ocelot won Mexico Robocup soccer Open second place and achieved multiple national awards in programming and robotics.

> "I enjoy setting nearly impossible goals to push myself while learning. I believe that learning should always lead to building something real."

---

### Diego Vitales Medellín
**Age:** 22\
**Role:** Coach

I've been involved in robotics for 14+ years being a programmer for most of the projects I've taken part in. I've had may regional, national and international experiences. Now I'm working in sharing my knowledge with more people to push further their level and potential as well as helping them achieve their goals and find their passion.

> "I like to face challenges and even more so when it's with more people. Learning and creating something is better when shared."

---

## The Challenge <a name="the-challenge"></a>

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

## Robot Overview <a name="robot-overview"></a>

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

## Mobility Management <a name="mobility-management"></a>


### Drivetrain

**Design Notes:**
We opted for custom-built steel axles with a diameter of 4mm, connected to Pololu 25D 6V HP motors, which provided sufficient power and torque to meet our performancer requireeriments. The motors have a maximum RPM of, max spee 480, but theour torque output was initially too high for optimal speed. To address this, we ilemented  ado we decide use an extra reductional to 2:1 gear reduction, effectively doubling the speed while maintainingto have double speed and approximately 1.8 kg·cm of/ torque—suitable for our application.

The drivetrain base and gear assemblys were manufactured in-hby ourse. We used PLA Carbon Fiber filameona QIDI Q1 proprinter for extra strength and durability. We designed and printed d, and PLA Couble Helical Gears tld  energy transmission efficiency and reduce mechanical wear.

The steel axeis wer to size manually and shapemade by our self using an steel rod, we cut it at the needed size, and usinged a Ddremel tool to create D-shaped shafts, which ensured a secure grip with the wheel hubs. Our wheels were also custom-built using 3D-printed make D Shaft to ensure grip from our wheels, our wheels were also made using , using the same qidi, the made the tire rims, and motor shaft couplers sourcedused some motor coupler shaft we bugth from AaliEexpress. For tires, we repurposed LEGO rubber tires after determining that fabricating our own rubber tires was not viable.

**Motor:** Pololu 25D 6V HP
**Gear Ratio:** 20.4:1
**Max RPM:** 480

**Planned Improvements for National Phase:**
- Upgrade to Maxon DCX19 motors for better power-to-weight ratio.
- Implement a differential gear system for smoother cornering.
- Replace LEGO tires with higher-grip, custom-molded polyurethane ones., we used lego wheels tires as we realize that was to dificult to make our selfs tires, d us a remel to rim and  rom liess

**Motor:** Maxon DCX19
**Max RPM:** 600rpm

### Steering

**Servo Model:** HS 85mg
**Rotation Range:** 180°

We builded an stering system to ensure smooth and better turns, we used a HS 85mg servo motor, all the mechanism is mounted on a 3d printed base, and we assemble it on the robot chassis.

**Future Upgrades:**
- Improve steering angle
---

#### 1. Arduino Control System (C++)
The main control loop runs on the Teensy 4.0, handling:

**Core Features:**
- Real-time motor control with PID feedback
- State machine for autonomous navigation modes
- Sensor data fusion from IMU and OTOS
- UART communication protocol with vision systems

#### 2. Pi Camera 3v
Handles primary computer vision tasks:

**Features:**
- Real-time color blob detection for red/green cubes and parking zone
- Centroid calculation for object following

#### 3. LIDAR Sector Analysis System (Python)

Our LIDAR system provides 360° environmental awareness with sector-based analysis:

```python
class LidarSectorAnalyzer:
    def __init__(self):
        self.target_angles = [0, 90, 180, 270]  # Cardinal directions
        self.angle_tolerance = 5                 # ±5° sector width
        self.sector_data = {angle: [] for angle in self.target_angles}
```

**Key Features:**
- Real-time distance measurements at cardinal directions (0°, 90°, 180°, 270°)
- Statistical analysis with moving averages for noise reduction
- Quality filtering to exclude unreliable readings
- Continuous monitoring with configurable reporting intervals

**Applications:**
- Wall detection for parallel parking
- Obstacle distance verification
- Navigation corridor analysis
- Backup sensor for vision system failures

#### 4. OTOS Position Tracking (Python)

The Optical Tracking Odometry Sensor provides precise position and heading data:

```python
def runExample():
    myOtos = qwiic_otos.QwiicOTOS()
    myOtos.begin()
    myOtos.calibrateImu()
    myOtos.resetTracking()

    while True:
        myPosition = myOtos.getPosition()
        # Returns X, Y coordinates in inches and heading in degrees
```

**Capabilities:**
- Sub-millimeter position accuracy
- Real-time heading calculation
- IMU calibration for drift compensation
- Continuous tracking with 0.5s update rate

#### 5. Enhanced Color Detection System (Python)

Advanced color detection using PiCamera2 for improved reliability:

**Features:**
- HSV color space processing for better color separation
- Morphological operations for noise reduction
- Multi-threshold detection for red color (handles hue wraparound)
- Real-time FPS monitoring and performance optimization
- Automatic image capture for debugging

**Color Ranges:**
- **Blue cubes:** HSV(100-130, 80-255, 80-255)
- **Red cubes:** HSV(0-10, 80-255, 80-255) + HSV(170-180, 80-255, 80-255)

### Communication Protocols

#### Data Flow Architecture

1. **Sensor Acquisition Layer**
   - LIDAR: 360° distance data at 10Hz
   - OTOS: Position/heading at 2Hz
   - Camera: Color blobs at 30Hz
   - IMU: Orientation at 100Hz

2. **Processing Layer**
   - Sensor fusion algorithms
   - Computer vision processing
   - Statistical filtering
   - State estimation

3. **Control Layer**
   - PID motor control
   - Path planning algorithms
   - Decision state machine
   - Safety monitoring

4. **Hardware Interface Layer**
   - Motor driver commands
   - Servo positioning
   - LED indicators
   - Emergency stop

### Code Structure

```
/code/
├── arduino/
│   ├── main.cpp              # Main control loop
├── vision/
│   ├── main.py              # Main vision script
├── raspberry_pi/
│   ├── lidar_analyzer.py    # LIDAR sector analysis
│   ├── otos_reader.py       # Position tracking
```

### Performance Characteristics

- **Vision Processing:** 30 FPS color detection
- **Control Loop:** 100 Hz motor control updates
- **LIDAR Refresh:** 10 Hz environmental scanning
- **Position Update:** 2 Hz absolute positioning
- **Communication Latency:** <10ms between subsystems

### Development Tools

- **Debugging:** Real-time data logging and visualization
- **Calibration:** Automated sensor calibration routines
- **Testing:** Unit tests for critical algorithms
- **Simulation:** Virtual environment for algorithm development

---

## Obstacle Management <a name="obstacle-management"></a>

The robot detects and reacts to obstacles in real-time using multiple sensor modalities:

### Detection Methods
- **Primary:** Enhanced color detection via PiCamera2 system
- **Verification:** LIDAR distance measurements for obstacle confirmation and navigation
- **Backup:** OTOS position tracking for navigation consistency

### Response Algorithms
- **Dynamic turning decision system** based on cube color and position
- **Follow-the-object mode** with PID steering based on cube centroid
- **Multi-sensor verification** to reduce false positives
- **Adaptive speed control** based on obstacle proximity

---

## Construction Guide <a name="construction-guide"></a>
- in construcction

**STL Files Folder:** `3d-models/`

**Sections to complete:**
- Step 0: 3D printing
- Step 1: Steering system
- Step 2: Powertrain and motor mount
- Step 3: Electronic layout
- Step 4: Wiring
- Step 5: Upload firmware

## Construction Tools
- 3D Printer (Creality K2 Plus, QIDI Q1 Pro)
- Mini Electric Soldering Iron Kit TS101
-
- Dremel Tool
- Screwdriver Set Fanttik

## Cost Report <a name="cost-report"></a>

| Item                         | Qty | Unit Cost (MXN) | Total (MXN) |
|------------------------------|-----|------------------|-------------|
| Teensy 4.0                   | 1   | 800              | 800         |
| Raspberry Pi 5                | 1   | 2800             | 2800        |
| RPlidar C1                    | 1   | 2500             | 2500        |
| Raspberry Pi Camera 12mp V3   | 1   | 920              | 920         |
| Raspberry Pi 5 Camera Cable   | 1   | 64               | 64          |
| 2.2Ah LiPo 11.1V Battery     | 1   | 600              | 600         |
| 1Ah LiPo 3.3V Battery     | 1   | 70               | 70          |
| Maxon Motor DCX19            | 1   | 8500             | 8500        |
| HS85MG Micro Servo            | 1   | 2000             | 2000        |
| SparkFun OTOS                 | 1   | 2400             | 2400        |
| POLYMAKER PLA Filament (prototypes)    | -   | 1kg = 900        | 900         |
| POLYMAKER PLA-CF Filament (finals)     | -   | 0.5kg = 450       | 450         |
| Carbon Fiber                  | 1   | 2000             | 2000        |
| SMD Components & Misc.   | -   |         1500      | 1500        |
| PCB Manufacturing             | 1   | 800              | 800         |
| Spike Wheels (LEGO)         | 4   | 150              | 600         |
| EV3 Wheels (LEGO)          | 2   | 10              | 20         |
| **Total**                     |     |                  | **26924 MXN**|

---

## Resources <a name="resources"></a>

- [Chabots Main Site](https://www.chabots.mx)
- [WRO Future Engineers Rules PDF](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
- [GitHub Repos](https://github.com/chabotsmx) *(to be added)*

---

## License <a name="license"></a>

```
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software...
(Full license text here)
```

---

> *Document maintained by Chabots | Last updated: June 2025*

<!--stackedit_data:
eyJoaXN0b3J5IjpbMTcyMzM3ODYxNCwtMzc2NTM2MDM5LDM1ND
c4NDQyMCwxMjQ4Mzg0MTM1LC0yODM3NTcxNywtMTMyNzEwNTIy
MywxMjg3Nzk2NjQsLTQ4MTYzMzM4MF19
-->


> Written with [StackEdit](https://stackedit.io/).
