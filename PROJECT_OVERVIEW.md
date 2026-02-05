# Wheeled Humanoid Robot Project - Complete Package

## 📦 What You've Received

This complete ROS2 Jazzy project includes everything needed to build a wheeled humanoid robot with VLM (Vision Language Model) integration using Qwen2.5-Omni-3B.

### Package Contents

```
humanoid_project/
├── README.md                          # Project overview
├── SETUP_GUIDE.md                     # Detailed setup instructions
├── QUICK_REFERENCE.md                 # Command reference
├── WIRING_GUIDE.md                    # Hardware wiring diagrams
├── test_system.py                     # Automated system test
│
└── src/
    ├── humanoid_motor_control/        # Motor & encoder control
    │   ├── include/                   # C++ headers
    │   ├── src/                       # C++ source files
    │   │   ├── cytron_controller.cpp  # Motor driver interface
    │   │   ├── cytron_controller_node.cpp
    │   │   ├── encoder_node.cpp       # Encoder reading
    │   │   └── diff_drive_controller.cpp  # Odometry
    │   ├── config/                    # Configuration files
    │   ├── launch/                    # Launch files
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── humanoid_vlm_bridge/           # VLM integration
        ├── humanoid_vlm_bridge/       # Python package
        │   ├── vlm_node.py            # Qwen2.5-Omni integration
        │   └── intent_executor.py     # Intent → robot actions
        ├── config/                    # VLM configuration
        ├── launch/                    # Launch files
        ├── setup.py
        └── package.xml
```

---

## 🎯 Key Features

### ✅ Motor Control System
- **Cytron MDDRC10 driver integration** with 4 DC motors
- **Optical encoder support** for odometry
- **Differential drive kinematics** for wheeled base
- **ROS2 Twist command interface** (cmd_vel)
- **Emergency stop functionality**

### ✅ VLM Integration
- **Qwen2.5-Omni-3B** local VLM model
- **Vision + text input** processing
- **Intent extraction** from VLM responses
- **Automatic action execution** based on intents
- **Multi-modal interaction** (camera + text/audio)

### ✅ Navigation & Odometry
- **Real-time odometry** from wheel encoders
- **TF2 transform publishing**
- **nav_msgs/Odometry** output for nav2 integration
- **IMU ready** (MPU6050 support structure)

### ✅ Distributed Architecture
- **Raspberry Pi 4**: Motor control, sensors, encoders
- **Gaming PC**: VLM processing, high-level planning
- **ROS2 DDS**: Automatic cross-machine communication
- **Android tablet ready**: For UI/display interface

---

## 🚀 Quick Start (30 Minutes)

### Step 1: Hardware Assembly (15 min)
1. **Connect motors** to Cytron MDDRC10
2. **Wire encoders** to Raspberry Pi GPIO (see WIRING_GUIDE.md)
3. **Connect Cytron** to Pi via serial (TX/RX)
4. **Attach MPU6050** via I2C
5. **Connect cameras** (USB or CSI)
6. **Power up** using buck converter for Pi

📖 **Detailed instructions**: See `WIRING_GUIDE.md`

### Step 2: Raspberry Pi Setup (10 min)
```bash
# Install ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# Create workspace
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src

# Copy humanoid_motor_control package here

# Build
cd ~/humanoid_ws
colcon build --packages-select humanoid_motor_control
source install/setup.bash

# Configure network
export ROS_DOMAIN_ID=42
```

📖 **Detailed instructions**: See `SETUP_GUIDE.md`

### Step 3: Gaming PC Setup (5 min)
```bash
# Install dependencies
pip3 install torch transformers opencv-python

# Download Qwen2.5-Omni model
mkdir -p ~/models
# Clone from HuggingFace

# Build VLM package
cd ~/humanoid_ws
colcon build --packages-select humanoid_vlm_bridge
source install/setup.bash

# Same network settings
export ROS_DOMAIN_ID=42
```

### Step 4: Test! (5 min)
```bash
# On Raspberry Pi
ros2 launch humanoid_motor_control motor_control.launch.py

# On Gaming PC
ros2 launch humanoid_vlm_bridge vlm_bridge.launch.py

# Test movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Run automated tests
python3 test_system.py
```

---

## 📊 System Architecture

### Data Flow
```
Camera → VLM Node → Intent → Intent Executor → cmd_vel → Motor Controller → Motors
   ↓                                                                            ↓
Image Topic                                                                 Encoders
                                                                               ↓
                                                                     Diff Drive Controller
                                                                               ↓
                                                                            Odometry
```

### Topic Graph
```
Subscribed Topics:
  /cmd_vel              - Velocity commands to robot
  /camera/image_raw     - Vision input for VLM
  /vlm_text_input       - Text queries for VLM
  /vlm_audio_input      - Audio input (transcribed)
  /emergency_stop       - E-stop trigger

Published Topics:
  /odom                 - Robot odometry
  /joint_states         - Wheel joint states  
  /wheel_velocities     - Raw wheel speeds
  /vlm_response         - VLM text responses
  /vlm_intent           - Extracted action intents
  /robot_status         - Current robot state
  /vlm_ready            - VLM system status
```

---

## ⚙️ Configuration

### Motor Control Parameters
Edit: `src/humanoid_motor_control/config/motor_control.yaml`

```yaml
serial_port: "/dev/ttyUSB0"     # Cytron connection
wheel_base: 0.40                # Distance between wheels (m)
wheel_radius: 0.05              # Wheel radius (m)
max_speed: 255                  # Max PWM value
encoder_pins: [17, 18, 27, 22]  # GPIO pins
```

### VLM Parameters
Edit: `src/humanoid_vlm_bridge/config/vlm_config.yaml`

```yaml
model_path: "~/models/Qwen2.5-Omni-3B"
device: "cuda"                  # or "cpu"
linear_speed: 0.3               # Movement speed (m/s)
angular_speed: 0.5              # Turn speed (rad/s)
process_rate: 2.0               # VLM processing frequency (Hz)
```

---

## 🎮 Usage Examples

### Basic Movement
```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Backward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

### VLM Interaction
```bash
# Ask what the robot sees
ros2 topic pub /vlm_text_input std_msgs/msg/String \
  "data: 'What objects do you see in front of you?'"

# Give a command
ros2 topic pub /vlm_text_input std_msgs/msg/String \
  "data: 'Move forward and avoid obstacles'"

# Execute specific intent
ros2 topic pub /vlm_intent std_msgs/msg/String "data: 'wave'"
```

### Monitoring
```bash
# Watch odometry
ros2 topic echo /odom

# Monitor VLM responses
ros2 topic echo /vlm_response

# Check system status
ros2 topic echo /robot_status
```

---

## 🔧 Customization & Extensions

### Adding New Intents
Edit `humanoid_vlm_bridge/vlm_node.py`:

```python
def extract_intent(self, response: str) -> Optional[str]:
    intents = {
        'move_forward': ['move forward', 'go ahead'],
        'your_new_intent': ['your', 'trigger', 'words'],
    }
    # ...
```

Edit `humanoid_vlm_bridge/intent_executor.py`:

```python
def execute_intent(self, intent: str):
    if intent == 'your_new_intent':
        self.your_new_action()
```

### Adjusting Motor Speeds
Tune in `motor_control.yaml` or at runtime:

```bash
ros2 param set /cytron_controller max_speed 200
ros2 param set /diff_drive_controller wheel_base 0.42
```

### Adding Sensors
1. Create new node in `humanoid_motor_control/src/`
2. Add to CMakeLists.txt
3. Include in launch file
4. Publish to relevant topics

---

## 🐛 Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| Motors not moving | Check serial connection, verify `/dev/ttyUSB0` exists, add user to `dialout` group |
| Encoders not counting | Verify GPIO pins, check encoder power (3.3V), test with LED |
| VLM not responding | Check CUDA with `nvidia-smi`, verify model path, check camera feed |
| Nodes can't communicate | Verify `ROS_DOMAIN_ID` matches on all machines, check network connectivity |
| High latency | Use wired Ethernet, reduce VLM `process_rate`, lower camera resolution |

📖 **Full troubleshooting guide**: See `SETUP_GUIDE.md` and `QUICK_REFERENCE.md`

---

## 📚 Documentation Files

| File | Purpose |
|------|---------|
| `README.md` | Project overview and hardware list |
| `SETUP_GUIDE.md` | Complete installation and setup instructions |
| `QUICK_REFERENCE.md` | Commands, topics, and quick troubleshooting |
| `WIRING_GUIDE.md` | Detailed hardware wiring diagrams |
| `test_system.py` | Automated system testing script |

---

## 🎓 Learning Path

### Beginner (Week 1)
1. Assemble hardware following WIRING_GUIDE.md
2. Install ROS2 and build packages
3. Test basic motor movement
4. Verify encoder readings

### Intermediate (Week 2)
1. Set up VLM on gaming PC
2. Test camera integration
3. Send text queries to VLM
4. Configure intent executor

### Advanced (Week 3+)
1. Integrate with nav2 for autonomous navigation
2. Add person detection/tracking
3. Implement SLAM for mapping
4. Create Android UI for interaction
5. Add arm servos when ready

---

## ⚠️ Important Notes

### Safety
- **Test in open space** - give robot room to move
- **Emergency stop** - keep accessible at all times
- **Power management** - use separate supplies for Pi and motors
- **Monitor battery** - don't over-discharge

### Performance
- **Network**: Wired Ethernet recommended for reliable communication
- **VLM speed**: Adjust `process_rate` based on your GPU
- **Encoder accuracy**: Calibrate `pulses_per_revolution` experimentally

### Known Limitations
1. **Cytron protocol**: Code uses simplified serial protocol - verify with datasheet
2. **Encoder implementation**: Currently single-channel; upgrade to quadrature for direction detection
3. **IMU integration**: MPU6050 wiring ready but sensor fusion not implemented
4. **VLM latency**: ~0.5-2s depending on GPU (normal for local inference)

---

## 🚀 Next Steps

### Immediate (This Week)
- [ ] Complete hardware assembly
- [ ] Install software on both machines
- [ ] Run `test_system.py`
- [ ] Calibrate wheel parameters
- [ ] Test VLM interaction

### Short-term (This Month)
- [ ] Add IMU sensor fusion
- [ ] Implement person following
- [ ] Create Android UI
- [ ] Add voice input/output
- [ ] Tune PID control for motors

### Long-term (This Quarter)
- [ ] Add manipulator arms
- [ ] Integrate SLAM and nav2
- [ ] Implement object detection
- [ ] Add gesture recognition
- [ ] Build complete interaction scenarios

---

## 🤝 Support & Resources

### ROS2 Resources
- Official docs: https://docs.ros.org/en/jazzy/
- ROS2 tutorials: https://docs.ros.org/en/jazzy/Tutorials.html
- ROS Answers: https://answers.ros.org/

### Hardware Resources
- Cytron MDDRC10: [Cytron website]
- Raspberry Pi GPIO: https://pinout.xyz/
- Qwen2.5-Omni: https://huggingface.co/Qwen/Qwen2.5-Omni-3B

### Community
- ROS Discourse: https://discourse.ros.org/
- Reddit: r/ROS, r/robotics
- Discord: ROS2 community servers

---

## 📝 Project Status

✅ **Completed**
- Motor control architecture
- VLM integration framework
- Encoder odometry system
- Launch file configuration
- Documentation suite

🔄 **In Progress** (Your Tasks)
- Hardware assembly
- Parameter calibration
- Testing and validation

🎯 **Future Enhancements**
- IMU sensor fusion
- SLAM integration
- Computer vision pipelines
- Arm/gripper control
- Advanced behaviors

---

## 📄 License

This project is provided under the MIT License. Feel free to modify, extend, and share!

---

## 🙏 Acknowledgments

Built with:
- **ROS2 Jazzy** - Robot Operating System
- **Qwen2.5-Omni** - Vision Language Model by Alibaba
- **Cytron MDDRC10** - Motor driver
- **Raspberry Pi** - Single-board computer

---

**Ready to build your humanoid robot? Start with `SETUP_GUIDE.md`!** 🤖
