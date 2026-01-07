# ROS Sensor Noise Package

This package provides noise injection for odometry sensor data in ROS 2.

## Features

- **Position Noise**: Gaussian noise injection for x, y, z position
- **Linear Velocity Noise**: Gaussian noise for linear velocity components
- **Quaternion Noise**: Noise injection using angle-axis representation
  - Converts quaternion to angle-axis
  - Injects noise into angle and axis components
  - Normalizes the axis vector
  - Converts back to quaternion
- **Angular Velocity Noise**: Gaussian noise for angular velocity components

## Usage

### Launch the Node

```bash
ros2 launch ros_sensor_noise odom_noise.launch.py
```

### Run the Node Directly

```bash
ros2 run ros_sensor_noise ros_odom_noise_generator --ros-args --params-file /path/to/noise.yaml
```

## Configuration

Edit `config/noise.yaml` to adjust noise parameters:

```yaml
/**:
  ros__parameters:
    noise:
      position_stddev: 0.002        # Position noise (meters)
      angle_stddev: 0.01            # Angle noise (radians)
      axis_stddev: 0.005            # Axis noise (radians)
      linear_velocity_stddev: 0.01  # Linear velocity noise (m/s)
      angular_velocity_stddev: 0.01 # Angular velocity noise (rad/s)
      apply_noise: true             # Enable/disable noise
```

## Topics

### Subscribed Topics
- `/odom` (nav_msgs/Odometry): Input odometry data

### Published Topics
- `/noisy_odom` (nav_msgs/Odometry): Odometry data with noise injected

## Implementation Details

### Quaternion Noise Injection

The quaternion noise injection follows these steps:
1. Convert quaternion to Eigen quaternion
2. Convert to angle-axis representation
3. Apply Gaussian noise to the angle
4. Apply Gaussian noise to each axis component
5. Normalize the axis vector
6. Create new angle-axis with noisy values
7. Convert back to quaternion
8. Publish the noisy quaternion

This ensures that the resulting quaternion is still valid and normalized.
