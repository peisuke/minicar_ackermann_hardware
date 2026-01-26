# minicar_ackermann_hardware

ROS2 Control Hardware Interface for Ackermann Steering Robot (TT-02 Chassis)

## Hardware Configuration

- **Chassis**: TAMIYA TT-02
- **Motor**: TT-02 540 Brushed Motor
- **ESC**: TAMIYA ESC 04S (45069)
- **Steering Servo**: SAVOX SC-1251MG PLUS
- **PWM Control**: OSOYOO PWM HAT (PCA9685)
- **SBC**: Raspberry Pi
- **LiDAR**: RPLiDAR C1

## PWM Signal Specification

| Device | Frequency | Neutral | Range |
|--------|-----------|---------|-------|
| ESC | 50Hz | 1.5ms | 1.0-2.0ms |
| Servo | 50Hz | 1.5ms | 1.0-2.0ms |

## PCA9685 Channel Assignment

| Channel | Function |
|---------|----------|
| 0 | ESC (Throttle) |
| 1 | Steering Servo |

## Launch

### Full System (with LiDAR)
```bash
ros2 launch minicar_ackermann_hardware real_robot_bringup.launch.py
```

### Motor Only (no LiDAR)
```bash
ros2 launch minicar_ackermann_hardware motor_only.launch.py
```

## Control Topic

```bash
# Send velocity command
ros2 topic pub /real_robot/ackermann_steering_controller/reference_unstamped \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

## Configuration

### Hardware Parameters (minicar_ackermann_real.xacro)
- `wheel_radius`: Wheel radius in meters (default: 0.032)
- `max_wheel_velocity_rad_per_sec`: Max wheel angular velocity
- `max_steering_angle_rad`: Max steering angle in radians
- `servo_*_pulse_ms`: Servo PWM calibration
- `esc_*_pulse_ms`: ESC PWM calibration

### Controller Parameters (ackermann_controller.yaml)
- `wheelbase`: Distance between front and rear axles
- `front_wheel_track`: Distance between front wheels
- `rear_wheel_track`: Distance between rear wheels
