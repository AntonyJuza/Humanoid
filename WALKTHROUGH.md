# rover_navigation — Complete Walkthrough

## Package Overview

```
rover_navigation/
├── rover_navigation/
│   ├── __init__.py
│   ├── ultrasonic_sensor_node.py    ← reads 3 HC-SR04s → publishes distances
│   └── navigation_node.py           ← subscribes distances + fire → publishes /cmd_vel
├── config/
│   └── navigation.yaml              ← all tunable params (pins, speeds, thresholds)
├── launch/
│   └── navigation.launch.py         ← starts both nodes together
├── ultrasonic_urdf_addition.urdf    ← paste into your existing URDF
├── package.xml
└── setup.py
```

---

## How It All Connects

```
[HC-SR04 x3]
    │  GPIO (BCM)
    ▼
[ultrasonic_sensor_node]
    │  /ultrasonic/distances  (Float32MultiArray: [front, front_left, front_right] cm)
    │  /ultrasonic/front      (Float32, individual — for debugging)
    │  /ultrasonic/front_left
    │  /ultrasonic/front_right
    ▼
[navigation_node] ◄── /fire_detected   (Bool)     from your YOLO/VLM server
                  ◄── /fire_direction  (String)   "left"|"center"|"right"
                  ◄── /emergency_stop  (Bool)     mirrors motor pkg e-stop
                  │
                  │  /cmd_vel  (geometry_msgs/Twist)
                  ▼
[cytron_controller_node]              ← humanoid_motor_control pkg (your existing C++ node)
    │  Converts: linear.x + angular.z → differential drive L/R wheel %
    │  left_vel  = linear - (angular × wheel_base/2)
    │  right_vel = linear + (angular × wheel_base/2)
    │  Mapped to -100..100 range, sent as PWM
    ▼
[Cytron Motor Driver]
    GPIO 12 (RC1) = Left motors   ← Hardware PWM0
    GPIO 13 (RC2) = Right motors  ← Hardware PWM1
```

---

## Step 1 — Hardware Wiring

### HC-SR04 to RPI

⚠️ HC-SR04 ECHO pin outputs 5V. RPI GPIO is only 3.3V tolerant.
Use a voltage divider on each ECHO pin:

```
ECHO pin ──── 1kΩ ──── GPIO (RPI) ──── 2kΩ ──── GND
```

This brings 5V down to ~3.3V safely.

```
Sensor          VCC    GND    TRIG        ECHO (via divider)
─────────────────────────────────────────────────────────────
Front Center    5V     GND    GPIO 5      GPIO 6
Front Left      5V     GND    GPIO 17     GPIO 27
Front Right     5V     GND    GPIO 22     GPIO 23
```

> All GPIO numbers are BCM. You can change them in config/navigation.yaml.

### Motor Driver (already wired — for reference)
```
Cytron RC1  ← GPIO 12  (Hardware PWM0) = Left motors
Cytron RC2  ← GPIO 13  (Hardware PWM1) = Right motors
```

---

## Step 2 — Install Dependencies on RPI

```bash
# RPi.GPIO (should already be present on RPI OS)
pip3 install RPi.GPIO --break-system-packages

# Verify ROS2 Jazzy is sourced
source /opt/ros/jazzy/setup.bash
```

---

## Step 3 — Create the Package in Your Workspace

```bash
cd ~/ros2_ws/src

# Create package directory
mkdir -p rover_navigation/rover_navigation
mkdir -p rover_navigation/config
mkdir -p rover_navigation/launch
mkdir -p rover_navigation/resource

# Copy all files from this repo into the structure above
# Then create the required __init__.py
touch rover_navigation/rover_navigation/__init__.py

# Create resource marker file (required by ament_python)
touch rover_navigation/resource/rover_navigation
```

Your workspace should look like:
```
ros2_ws/
└── src/
    ├── humanoid_motor_control/      ← your existing motor pkg
    └── rover_navigation/            ← new pkg
        ├── rover_navigation/
        │   ├── __init__.py
        │   ├── ultrasonic_sensor_node.py
        │   └── navigation_node.py
        ├── config/navigation.yaml
        ├── launch/navigation.launch.py
        ├── resource/rover_navigation
        ├── package.xml
        └── setup.py
```

---

## Step 4 — Build

```bash
cd ~/ros2_ws

# Build only the new package first
colcon build --packages-select rover_navigation

# Source the workspace
source install/setup.bash
```

If there are no errors, you're ready.

---

## Step 5 — Measure and Set wheel_base

Before running navigation, measure your actual wheel base:

```
wheel_base = distance between the CENTER of left wheel and CENTER of right wheel
```

From your URDF: wheels are at y=±0.1175, so wheel_base = 0.235m.
Measure the physical rover to confirm — update config/navigation.yaml if different.
This MUST match the wheel_base in your motor pkg yaml too.

---

## Step 6 — Run

### Terminal 1: Start pigpiod (required by motor pkg)
```bash
sudo pigpiod
```

### Terminal 2: Start motor node (your existing pkg)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run humanoid_motor_control cytron_controller_node \
  --ros-args --params-file ~/ros2_ws/src/humanoid_motor_control/config/motor_control.yaml
```

### Terminal 3: Start navigation stack
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rover_navigation navigation.launch.py
```

---

## Step 7 — Testing Sensor Readings First

Before testing navigation, verify sensors are working:

```bash
# Watch combined array [front, front_left, front_right] in cm
ros2 topic echo /ultrasonic/distances

# Watch individual sensors
ros2 topic echo /ultrasonic/front
ros2 topic echo /ultrasonic/front_left
ros2 topic echo /ultrasonic/front_right
```

Expected output when path is clear:
```
data: [150.0, 140.0, 145.0]
```

Hold your hand 20cm in front of the front sensor:
```
data: [20.3, 140.0, 145.0]   ← front drops, sides unchanged
```

---

## Step 8 — Testing Navigation

### Manual mode switch test (fire approach)
```bash
# Simulate fire detected (center)
ros2 topic pub --once /fire_detected std_msgs/msg/Bool "data: true"
ros2 topic pub --once /fire_direction std_msgs/msg/String "data: center"

# Rover should move forward slowly
# Watch what cmd_vel it publishes:
ros2 topic echo /cmd_vel

# Simulate fire to the left
ros2 topic pub --once /fire_direction std_msgs/msg/String "data: left"
# Rover should curve left

# Clear fire detection
ros2 topic pub --once /fire_detected std_msgs/msg/Bool "data: false"
# Rover should return to SEARCH mode
```

### Emergency stop test
```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: true"
# Both navigation AND motor node will stop
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: false"
# Resumes
```

### Monitor nav status log (feeds your UI dashboard)
```bash
ros2 topic echo /nav_status
```

Example output:
```
data: "SEARCH — moving forward  F:120 FL:110 FR:115"
data: "SEARCH — front blocked (22cm), turning LEFT"
data: "APPROACH → fire CENTER  F:80cm  spd=0.25"
```

---

## Step 9 — Publishing fire_direction from Your CV Server

Your PC server (YOLO/VLM) needs to publish two topics to the RPI:

```python
# On your PC server (already running ROS2 Jazzy):
from std_msgs.msg import Bool, String

fire_pub = node.create_publisher(Bool, '/fire_detected', 10)
dir_pub  = node.create_publisher(String, '/fire_direction', 10)

# When YOLO detects fire and VLM validates:
fire_pub.publish(Bool(data=True))

# Determine direction from bounding box center x vs image width:
# bbox_center_x < image_width/3       → "left"
# bbox_center_x > 2 * image_width/3   → "right"
# else                                 → "center"
dir_pub.publish(String(data="center"))
```

---

## Tuning Guide

| Parameter         | Default | Increase if...                        | Decrease if...                    |
|-------------------|---------|---------------------------------------|-----------------------------------|
| danger_dist_cm    | 25      | Robot hits obstacles before stopping  | Robot stops too far from walls    |
| caution_dist_cm   | 50      | Turns feel too sharp/sudden           | Robot doesn't start turning early |
| forward_speed     | 0.25    | Robot moves too slowly                | Robot overshoots turns            |
| turn_speed        | 0.6     | Robot turns too slowly                | Robot spins too fast              |
| publish_rate_hz   | 10      | Readings feel laggy                   | RPI CPU load is high              |

---

## Common Issues

**Sensor reads always 200 (max)**
→ Check TRIG/ECHO wiring. Check voltage divider on ECHO.
→ Run `gpio readall` to confirm pins are set correctly.

**Sensor reads are very noisy**
→ Increase MOVING_AVG_SIZE in ultrasonic_sensor_node.py (default 5).
→ Add 15ms delay between sensor triggers (already in code).

**Robot doesn't stop at obstacles**
→ Lower `danger_dist_cm` in navigation.yaml.
→ Check sensor is mounted at correct height (not blocked by chassis).

**Navigation node starts but motors don't move**
→ Confirm cytron_controller_node is running: `ros2 node list`
→ Check /cmd_vel is being published: `ros2 topic echo /cmd_vel`
→ Check pigpiod is running: `ps aux | grep pigpiod`

**Cross-talk between sensors (erratic readings)**
→ The 15ms sequential delay in ultrasonic_sensor_node.py handles this.
→ If still an issue, increase the delay to 30ms.
