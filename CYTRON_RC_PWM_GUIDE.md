# Cytron MDDRC10 RC/PWM Mode Wiring Guide

## ⚡ IMPORTANT: RC PWM Signal Specifications

The Cytron MDDRC10 uses **RC/Servo PWM signals** for control, NOT serial communication!

### Signal Characteristics
- **Signal Type**: RC PWM (Pulse Width Modulation)
- **Frequency**: 50Hz (20ms period)
- **Pulse Width Range**: 1000μs to 2000μs
- **Neutral/Stop**: 1500μs ⚠️ **CRITICAL CALIBRATION POINT**
- **Direction Control**:
  - **1000μs - 1500μs**: Counter-clockwise (reverse)
  - **1500μs**: STOP (neutral position)
  - **1500μs - 2000μs**: Clockwise (forward)
- **Speed Control**: Distance from 1500μs determines speed
  - Closer to 1500μs = slower
  - Closer to 1000μs or 2000μs = faster

### 🔴 Critical Initialization Requirement
**On power-up, you MUST send 1500μs pulses for ~1 second to calibrate the neutral position!**
The code does this automatically in the `init()` function.

---

## Wiring Connections

### Complete Cytron MDDRC10 Connection Diagram

```
┌─────────────────────────────────────────────────────────┐
│                  Raspberry Pi 4                         │
│                                                          │
│  GPIO 23 (Pin 16) ─────────────┐                       │
│  GPIO 24 (Pin 18) ─────────┐   │                       │
│      GND (Pin 20) ─────┐   │   │                       │
│                        │   │   │                       │
└────────────────────────┼───┼───┼───────────────────────┘
                         │   │   │
                         │   │   │
                    ┌────▼───▼───▼──────────────────────┐
                    │   Cytron MDDRC10                  │
                    │                                   │
                    │   GND ◄─── GND                    │
                    │   RC1 ◄─── GPIO 23 (RC PWM)       │
                    │   RC2 ◄─── GPIO 24 (RC PWM)       │
                    │                                   │
                    │   VIN ◄─── 12V from Battery       │
                    │   GND ◄─── Battery GND            │
                    │                                   │
                    │   M1A ───► Motor 1 (Front Left)   │
                    │   M1B ───► Motor 3 (Rear Left)    │
                    │                                   │
                    │   M2A ───► Motor 2 (Front Right)  │
                    │   M2B ───► Motor 4 (Rear Right)   │
                    └───────────────────────────────────┘
```

### Pin-by-Pin Connections

#### Power Connections
```
Battery 12V (+) ──► Cytron VIN
Battery GND (-)  ──► Cytron GND
                 └─► Raspberry Pi GND (common ground)
```

#### Control Signal Connections
```
Raspberry Pi              Cytron MDDRC10
────────────              ──────────────
GPIO 23 (Pin 16) ───────► RC1 (left motors control)
GPIO 24 (Pin 18) ───────► RC2 (right motors control)
GND      (Pin 20) ───────► GND (signal ground)
```

#### Motor Connections (Cytron → Motors)
```
Cytron Output        Motor
─────────────        ─────
M1A (+/-)      ────► Motor 1 (Front Left)
M1B (+/-)      ────► Motor 3 (Rear Left)
M2A (+/-)      ────► Motor 2 (Front Right)
M2B (+/-)      ────► Motor 4 (Rear Right)
```

**Note**: The Cytron MDDRC10 controls motor direction internally based on the RC signal. You don't need to worry about swapping motor polarity - the driver handles it.

---

## Raspberry Pi GPIO Pinout (for RC PWM)

```
   3V3  (1)  (2)  5V
   SDA  (3)  (4)  5V
   SCL  (5)  (6)  GND
       (7)  (8)  TXD
   GND  (9) (10)  RXD
GPIO17 (11)(12) GPIO18  ← Encoder 2
GPIO27 (13)(14) GND
GPIO22 (15)(16) GPIO23  ← RC1 (Left Motors)
   3V3 (17)(18) GPIO24  ← RC2 (Right Motors)
      (19)(20) GND      ← Common Ground
      (21)(22) GPIO25
      (23)(24)
   GND (25)(26)
```

---

## Testing the RC PWM Signals

### Quick Test Script (Python)

Save as `test_rc_pwm.py`:

```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# GPIO pins
RC1_PIN = 23  # Left motors
RC2_PIN = 24  # Right motors

# PWM constants (microseconds)
PWM_MIN = 1000
PWM_NEUTRAL = 1500
PWM_MAX = 2000

GPIO.setmode(GPIO.BCM)
GPIO.setup(RC1_PIN, GPIO.OUT)
GPIO.setup(RC2_PIN, GPIO.OUT)

def send_rc_pulse(pin, pulse_us):
    """Send a single RC PWM pulse"""
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(pulse_us / 1000000.0)  # Convert to seconds
    GPIO.output(pin, GPIO.LOW)
    time.sleep((20000 - pulse_us) / 1000000.0)  # Complete 20ms period

try:
    print("Initializing with neutral position (1500μs)...")
    for _ in range(50):  # 1 second at 50Hz
        send_rc_pulse(RC1_PIN, PWM_NEUTRAL)
        send_rc_pulse(RC2_PIN, PWM_NEUTRAL)
    
    print("Testing forward motion (1700μs)...")
    for _ in range(100):  # 2 seconds
        send_rc_pulse(RC1_PIN, 1700)
        send_rc_pulse(RC2_PIN, 1700)
    
    print("Back to neutral (1500μs)...")
    for _ in range(50):
        send_rc_pulse(RC1_PIN, PWM_NEUTRAL)
        send_rc_pulse(RC2_PIN, PWM_NEUTRAL)
    
    print("Testing reverse motion (1300μs)...")
    for _ in range(100):
        send_rc_pulse(RC1_PIN, 1300)
        send_rc_pulse(RC2_PIN, 1300)
    
    print("Final neutral...")
    for _ in range(50):
        send_rc_pulse(RC1_PIN, PWM_NEUTRAL)
        send_rc_pulse(RC2_PIN, PWM_NEUTRAL)
    
    print("Test complete!")

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    # Send neutral before cleanup
    for _ in range(10):
        send_rc_pulse(RC1_PIN, PWM_NEUTRAL)
        send_rc_pulse(RC2_PIN, PWM_NEUTRAL)
    GPIO.cleanup()
```

Run with:
```bash
sudo python3 test_rc_pwm.py
```

### Expected Motor Behavior

| Pulse Width | Motor Behavior |
|-------------|----------------|
| 1000μs      | Full speed reverse (CCW) |
| 1250μs      | Half speed reverse |
| 1500μs      | STOP (neutral) |
| 1750μs      | Half speed forward (CW) |
| 2000μs      | Full speed forward |

---

## Motor Configuration

### Left vs Right Motor Channels

**RC1 Channel (Left Side)**:
- Controls both left motors together
- Motors connected to M1A and M1B outputs
- GPIO 23

**RC2 Channel (Right Side)**:
- Controls both right motors together
- Motors connected to M2A and M2B outputs
- GPIO 24

### Differential Drive Movement

| Movement    | RC1 (Left) | RC2 (Right) |
|-------------|------------|-------------|
| Forward     | 1700μs     | 1700μs      |
| Backward    | 1300μs     | 1300μs      |
| Turn Left   | 1300μs     | 1700μs      |
| Turn Right  | 1700μs     | 1300μs      |
| Rotate CCW  | 1300μs     | 1700μs      |
| Rotate CW   | 1700μs     | 1300μs      |
| Stop        | 1500μs     | 1500μs      |

---

## Code Mapping (Speed to PWM)

The controller converts speed percentage (-100 to +100) to PWM:

```
Speed %    →    PWM (μs)    →    Motor Action
────────────────────────────────────────────
 -100      →     1000        →    Full reverse
  -50      →     1250        →    Half reverse
    0      →     1500        →    STOP
  +50      →     1750        →    Half forward
 +100      →     2000        →    Full forward
```

Formula: `PWM = 1500 + (speed * 5)`

---

## Troubleshooting

### Motors Not Moving

1. **Check initialization**:
   ```bash
   # Monitor ROS logs
   ros2 run humanoid_motor_control cytron_controller_node
   # Should see: "Initializing Cytron MDDRC10 - sending neutral signals..."
   ```

2. **Verify GPIO output**:
   ```bash
   # Check if GPIO is outputting
   gpio -g mode 23 out
   gpio -g write 23 1
   gpio -g write 23 0
   ```

3. **Check power**:
   - Verify 12V on Cytron VIN
   - Check common ground between Pi and Cytron

### Motors Running at Wrong Speed

1. **Calibrate neutral position**: Ensure 1500μs initialization is working
2. **Adjust max_speed** in config: Lower if motors are too fast
3. **Check motor gear ratio**: May need different speed mapping

### One Side Not Working

1. **Swap RC channels**: Try swapping GPIO 23 and 24
2. **Check specific channel**: Test RC1 and RC2 independently
3. **Verify M1A/M1B and M2A/M2B connections**

### Jittery Movement

1. **PWM timing issue**: The bit-banging approach may have timing variations
2. **Solution**: Consider using hardware PWM pins (GPIO 12, 13, 18, 19)
3. **Alternative**: Use PCA9685 PWM driver (I2C) for more stable signals

---

## Hardware PWM Alternative (Recommended for Better Performance)

For more stable PWM signals, use hardware PWM pins:

```python
# Use hardware PWM (requires different GPIO pins)
# GPIO 18 (Pin 12) - Hardware PWM0
# GPIO 13 (Pin 33) - Hardware PWM1

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
pwm_rc1 = GPIO.PWM(18, 50)  # 50Hz
pwm_rc2 = GPIO.PWM(13, 50)

# Start with neutral (7.5% duty cycle = 1500μs)
pwm_rc1.start(7.5)
pwm_rc2.start(7.5)

# Duty cycle calculation:
# Duty% = (pulse_us / 20000) * 100
# For 1500μs: (1500/20000)*100 = 7.5%
# For 1700μs: (1700/20000)*100 = 8.5%
```

Update config to use hardware PWM pins:
```yaml
rc1_pin: 18  # Hardware PWM
rc2_pin: 13  # Hardware PWM
```

---

## Summary Checklist

Before starting:
- [ ] Cytron MDDRC10 powered with 12V
- [ ] Common ground between Pi and Cytron
- [ ] RC1 connected to GPIO 23 (or 18 for hardware PWM)
- [ ] RC2 connected to GPIO 24 (or 13 for hardware PWM)
- [ ] All motor connections secure
- [ ] Code sends 1500μs initialization sequence
- [ ] Test with simple script before ROS2

When running ROS2:
- [ ] Node initializes and sends neutral signals
- [ ] cmd_vel commands move robot correctly
- [ ] Emergency stop returns to 1500μs neutral
- [ ] No jitter or unexpected movements
