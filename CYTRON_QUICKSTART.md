# Quick Start: Cytron MDDRC10 RC PWM Setup

## 🎯 What Changed

Your Cytron MDDRC10 uses **RC/Servo PWM signals**, not serial! I've updated the entire motor control package to use the correct protocol.

## ⚡ Key Facts About RC PWM Control

- **Signal Type**: RC PWM (like RC car/airplane servos)
- **Pulse Width**: 1000-2000 microseconds
- **Neutral (STOP)**: 1500μs ← **CRITICAL!**
- **Forward**: 1500-2000μs (higher = faster)
- **Reverse**: 1000-1500μs (lower = faster)
- **Channels**: 
  - RC1 controls left motors (FL + RL)
  - RC2 controls right motors (FR + RR)

## 🔌 Wiring (UPDATED)

```
Raspberry Pi          Cytron MDDRC10
────────────          ──────────────
GPIO 23 (Pin 16) ───► RC1 (Left motors)
GPIO 24 (Pin 18) ───► RC2 (Right motors)
GND     (Pin 20) ───► GND
                      
12V Battery      ───► VIN
Battery GND      ───► GND
```

**NOT using serial (TX/RX) anymore!**

## 🧪 Testing Steps

### Step 1: Hardware Test (5 minutes)

1. **Wire the connections** as shown above
2. **Run the standalone test script**:
   ```bash
   sudo python3 test_cytron_rc_pwm.py
   ```
3. **Choose option 1** (automated test sequence)
4. **Watch the motors**:
   - Should initialize at neutral for 1 second
   - Forward, backward, turning tests
   - All smooth movements

If motors work ✅, proceed to Step 2!

### Step 2: Build Updated ROS2 Package (5 minutes)

```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_motor_control
source install/setup.bash
```

### Step 3: Test with ROS2 (2 minutes)

```bash
# Terminal 1: Launch motor controller
ros2 launch humanoid_motor_control motor_control.launch.py

# Terminal 2: Test movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 🔧 Configuration

The updated config file is at:
`src/humanoid_motor_control/config/motor_control.yaml`

```yaml
rc1_pin: 23  # GPIO for RC1 (left)
rc2_pin: 24  # GPIO for RC2 (right)
max_speed: 100  # Now a percentage (0-100)
```

**You can change the GPIO pins** if needed!

## 📊 Speed Mapping

The controller now converts speeds correctly:

| Your Command | Internal % | PWM Signal | Motor Action |
|--------------|------------|------------|--------------|
| cmd_vel: +0.3 | +60% | 1800μs | Forward |
| cmd_vel: -0.3 | -60% | 1200μs | Reverse |
| cmd_vel: 0.0 | 0% | 1500μs | STOP |

## 🐛 Troubleshooting

### Motors not moving?

1. **Check initialization message**:
   ```
   Should see: "Initializing Cytron MDDRC10 - sending neutral signals..."
   ```

2. **Verify GPIO pins**:
   ```bash
   # Test GPIO manually
   gpio -g mode 23 out
   gpio -g write 23 1
   gpio -g write 23 0
   ```

3. **Check power**: 12V on Cytron VIN pin

4. **Run standalone test**: `sudo python3 test_cytron_rc_pwm.py`

### Motors jittery or unstable?

The current code uses software PWM (bit-banging). For better performance:

**Option A: Use Hardware PWM pins** (recommended)
```yaml
# Change in motor_control.yaml:
rc1_pin: 18  # Hardware PWM0
rc2_pin: 13  # Hardware PWM1
```

**Option B: Use PCA9685 PWM driver** (most stable)
- I2C-based 16-channel PWM driver
- Rock-solid RC signals
- Can control 16 servos/motors

### Wrong direction?

If left/right are swapped:
```yaml
# Swap in motor_control.yaml:
rc1_pin: 24  # Was 23
rc2_pin: 23  # Was 24
```

If forward/backward is reversed, the code automatically handles it - no changes needed!

## 📚 What Files Changed

Updated files in your package:
- ✅ `cytron_controller.hpp` - RC PWM interface
- ✅ `cytron_controller.cpp` - RC PWM implementation
- ✅ `cytron_controller_node.cpp` - Uses GPIO pins instead of serial
- ✅ `motor_control.yaml` - GPIO pin configuration
- ✅ **NEW**: `test_cytron_rc_pwm.py` - Standalone test script
- ✅ **NEW**: `CYTRON_RC_PWM_GUIDE.md` - Detailed RC PWM guide

## 🚀 Next Steps After Testing

Once motors work with ROS2:

1. **Tune the speeds**: Adjust `max_speed` in config
2. **Calibrate wheel parameters**: Measure actual `wheel_base` and `wheel_radius`
3. **Add encoders**: Continue with encoder setup
4. **Test with VLM**: Full system integration

## 📖 Complete Documentation

- **Detailed RC PWM guide**: See `CYTRON_RC_PWM_GUIDE.md`
- **Full wiring diagrams**: See `WIRING_GUIDE.md` (updated)
- **Quick commands**: See `QUICK_REFERENCE.md`

---

**Ready to test? Start with the standalone script, then move to ROS2!** 🚀
