# Wheeled Humanoid Robot - ROS2 Jazzy Project

## Hardware Setup
- **Motors**: 4x Johnson Geared DC Motors
- **Motor Driver**: Cytron MDDRC10
- **Encoders**: Optical Photoelectric Speed Sensor Encoder Wheel
- **IMU**: MPU6050
- **Cameras**: RPi Camera, Webcam
- **VLM**: Qwen2.5-Omni-3B (local on Gaming PC)
- **Compute**: 
  - Gaming PC (VLM + ROS2 main processing)
  - Raspberry Pi 4 8GB (Motor control + sensors)
  - Android Tablet (UI/Display)

## Package Structure
```
humanoid_ws/
├── src/
│   ├── humanoid_description/      # URDF and robot description
│   ├── humanoid_motor_control/    # Cytron MDDRC10 driver
│   ├── humanoid_vlm_bridge/       # Qwen2.5-Omni integration
│   ├── humanoid_sensors/          # MPU6050, encoders, cameras
│   ├── humanoid_bringup/          # Launch files
│   └── humanoid_msgs/             # Custom messages
```

## Network Setup
- Gaming PC: ROS2 Domain ID, main processing
- Raspberry Pi 4: Motor control, sensor reading
- Communication: ROS2 DDS over WiFi/Ethernet

## Installation Steps
See individual package READMEs for detailed setup instructions.
