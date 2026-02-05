# Wheeled Humanoid Robot - Complete Setup Guide

## Hardware Overview

### Components
- **4x Johnson Geared DC Motors** - Wheel drive motors
- **Cytron MDDRC10** - Motor driver board
- **Optical Photoelectric Speed Sensor Encoder Wheels** - Odometry
- **MPU6050** - IMU sensor
- **RPi Camera + Webcam** - Vision input
- **Qwen2.5-Omni-3B** - Vision Language Model
- **Raspberry Pi 4 8GB** - Motor control & sensors
- **Gaming PC** - VLM processing
- **Android Tablet** - UI/Display

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Gaming PC (Ubuntu)                    │
│  ┌────────────────────────────────────────────────────┐ │
│  │          Qwen2.5-Omni VLM (CUDA)                   │ │
│  │  - vlm_node (image → intent)                       │ │
│  │  - intent_executor (intent → robot commands)       │ │
│  └────────────────────────────────────────────────────┘ │
│                         ↕ ROS2 DDS                       │
└─────────────────────────────────────────────────────────┘
                          ↕ WiFi/Ethernet
┌─────────────────────────────────────────────────────────┐
│              Raspberry Pi 4 (Ubuntu 24.04)              │
│  ┌────────────────────────────────────────────────────┐ │
│  │  Motor Control Nodes:                              │ │
│  │  - cytron_controller_node (cmd_vel → motors)       │ │
│  │  - encoder_node (encoders → velocities)            │ │
│  │  - diff_drive_controller (velocities → odom)       │ │
│  └────────────────────────────────────────────────────┘ │
│                    ↕ GPIO/Serial                         │
│  ┌────────────────────────────────────────────────────┐ │
│  │  Hardware:                                          │ │
│  │  - Cytron MDDRC10 (Serial/PWM)                     │ │
│  │  - 4x Encoders (GPIO)                              │ │
│  │  - MPU6050 (I2C)                                   │ │
│  │  - Cameras (USB/CSI)                               │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
                          ↕ WiFi
┌─────────────────────────────────────────────────────────┐
│                  Android Tablet (UI)                     │
│  - Web interface / ROS2 Android app                     │
│  - Display VLM responses                                │
│  - Voice input interface                                │
└─────────────────────────────────────────────────────────┘
```

## Prerequisites

### Raspberry Pi 4 Setup

```bash
# 1. Install Ubuntu 24.04 Server (64-bit) on Raspberry Pi
# Download from: https://ubuntu.com/download/raspberry-pi

# 2. Update system
sudo apt update && sudo apt upgrade -y

# 3. Install ROS2 Jazzy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop -y

# 4. Install dependencies
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    wiringpi \
    i2c-tools \
    python3-smbus

# 5. Initialize rosdep
sudo rosdep init
rosdep update

# 6. Setup environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 7. Install Python packages
pip3 install --break-system-packages \
    opencv-python \
    numpy \
    smbus2
```

### Gaming PC Setup (Ubuntu)

```bash
# 1. Install ROS2 Jazzy (same steps as Pi)

# 2. Install CUDA and PyTorch
# Visit: https://pytorch.org/get-started/locally/
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# 3. Install Transformers and dependencies
pip3 install transformers accelerate sentencepiece protobuf

# 4. Install additional packages
pip3 install opencv-python pillow
```

## Hardware Connections

### Cytron MDDRC10 Wiring

The Cytron MDDRC10 supports multiple control modes. For simplicity, we'll use **Serial/UART mode**.

#### Raspberry Pi → Cytron MDDRC10
```
RPi Pin        → Cytron Pin
GPIO 14 (TXD)  → RX
GPIO 15 (RXD)  → TX
GND            → GND
```

**Note**: The code currently uses a simplified serial protocol. You may need to adjust the `sendCommand()` function in `cytron_controller.cpp` based on the actual Cytron MDDRC10 protocol. Consult the Cytron datasheet.

### Encoder Connections

```
Encoder        → RPi GPIO Pin
Encoder 1      → GPIO 17 (Pin 11)
Encoder 2      → GPIO 18 (Pin 12)
Encoder 3      → GPIO 27 (Pin 13)
Encoder 4      → GPIO 22 (Pin 15)
VCC (all)      → 3.3V
GND (all)      → GND
```

### MPU6050 (IMU)

```
MPU6050        → RPi Pin
VCC            → 3.3V
GND            → GND
SDA            → GPIO 2 (Pin 3)
SCL            → GPIO 3 (Pin 5)
```

Enable I2C:
```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

## Building the Project

### On Raspberry Pi

```bash
# 1. Create workspace
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src

# 2. Clone/copy the packages (motor control)
# Copy humanoid_motor_control package here

# 3. Install dependencies
cd ~/humanoid_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --packages-select humanoid_motor_control

# 5. Source the workspace
source install/setup.bash
echo "source ~/humanoid_ws/install/setup.bash" >> ~/.bashrc
```

### On Gaming PC

```bash
# 1. Create workspace
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src

# 2. Copy VLM bridge package
# Copy humanoid_vlm_bridge package here

# 3. Download Qwen2.5-Omni model
mkdir -p ~/models
cd ~/models
# Download model (example - adjust based on actual model location)
git lfs install
git clone https://huggingface.co/Qwen/Qwen2.5-Omni-3B

# 4. Install dependencies
cd ~/humanoid_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --packages-select humanoid_vlm_bridge

# 6. Source workspace
source install/setup.bash
echo "source ~/humanoid_ws/install/setup.bash" >> ~/.bashrc
```

## Network Configuration

Both machines need to be on the same network and configured for ROS2 DDS communication.

### On Both Machines

```bash
# Set ROS_DOMAIN_ID (use same ID on both)
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# For better performance, configure DDS
# Create: ~/.ros/cyclonedds.xml
```

Create `~/.ros/cyclonedds.xml`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
```

```bash
echo "export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml" >> ~/.bashrc
source ~/.bashrc
```

## Running the System

### Step 1: Start Motor Control (Raspberry Pi)

```bash
# Terminal 1 - Launch motor controllers
ros2 launch humanoid_motor_control motor_control.launch.py

# Verify topics
ros2 topic list
# Should see:
# /cmd_vel
# /joint_states
# /odom
# /wheel_velocities
```

### Step 2: Start VLM Bridge (Gaming PC)

```bash
# Terminal 1 - Launch VLM nodes
ros2 launch humanoid_vlm_bridge vlm_bridge.launch.py

# Terminal 2 - Start camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
# Or for RPi camera:
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

### Step 3: Test the System

```bash
# Test 1: Manual robot control
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Test 2: Send VLM query
ros2 topic pub /vlm_text_input std_msgs/msg/String "data: 'What do you see?'"

# Test 3: Send movement intent
ros2 topic pub /vlm_intent std_msgs/msg/String "data: 'move_forward'"

# Test 4: Emergency stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true"
```

## Troubleshooting

### Motor Control Issues

**Motors not responding:**
1. Check serial connection: `ls -l /dev/ttyUSB*` or `/dev/serial0`
2. Verify permissions: `sudo usermod -a -G dialout $USER` (logout/login)
3. Test serial: `sudo minicom -D /dev/ttyUSB0 -b 9600`

**Encoders not working:**
1. Check GPIO connections
2. Test I/O: `gpio readall` (if using wiringPi)
3. Verify pin numbers in config file

### VLM Issues

**Model not loading:**
1. Check CUDA: `nvidia-smi`
2. Verify PyTorch: `python3 -c "import torch; print(torch.cuda.is_available())"`
3. Check model path in config file

**No camera feed:**
1. List cameras: `v4l2-ctl --list-devices`
2. Test camera: `ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw`

### Network Issues

**Nodes not communicating:**
1. Verify domain ID: `echo $ROS_DOMAIN_ID` (should be same on both)
2. Check firewall: `sudo ufw allow 7400:7500/udp`
3. Test connectivity: `ros2 topic list` (run on both machines)
4. Check IP: `hostname -I`

## Next Steps

1. **Tune Motor Parameters**: Adjust `wheel_base`, `wheel_radius`, `max_speed` in config files based on actual measurements and testing.

2. **Calibrate Encoders**: Test encoder counts and adjust `pulses_per_revolution`.

3. **Add IMU Integration**: Create node for MPU6050 to improve odometry.

4. **Create Android UI**: Develop app for voice input and VLM response display.

5. **Add SLAM**: Integrate `slam_toolbox` for mapping and localization.

6. **Implement Person Tracking**: Add face/person detection for follow mode.

7. **Add Arm Control**: When ready to add arms, create servo controller nodes.

## Important Notes

⚠️ **Cytron MDDRC10 Protocol**: The current implementation uses a simplified serial protocol. You MUST verify the actual protocol from Cytron's datasheet and update the `sendCommand()` function accordingly.

⚠️ **Encoder Type**: The code assumes simple pulse counting. If you have quadrature encoders, you'll need to modify the encoder node to read both A and B channels.

⚠️ **Safety**: Always test motor movements in a safe, open area. Have the emergency stop readily available.

⚠️ **Power Supply**: Ensure adequate power supply for motors. Separate power for Pi and motors is recommended.

## Support & Resources

- ROS2 Jazzy Documentation: https://docs.ros.org/en/jazzy/
- Cytron MDDRC10 Datasheet: [Cytron website]
- Qwen2.5-Omni: https://huggingface.co/Qwen/Qwen2.5-Omni-3B

## License

MIT License - See individual package files for details.
