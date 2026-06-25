# ROS Sensor Noise Package

This package provides noise injection for odometry sensor data in ROS 2.

## Features

- **Position Noise**: Gaussian noise injection for x, y, z position
- **Linear Velocity Noise**: Per-axis Gaussian noise for linear velocity (x, y, z)
- **Quaternion Noise**: Noise injection using angle-axis representation
  - Converts quaternion to angle-axis
  - Injects noise into angle and axis components
  - Normalizes the axis vector
  - Converts back to quaternion
- **Angular Velocity Noise**: Per-axis Gaussian noise for angular velocity (x, y, z)
- **Z Offset**: Rigid-body position and velocity offset from base_link to sensor (e.g., IMU) link
- **Body Frame Velocity**: Linear velocity is rotated from world frame to body frame before publishing

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

Edit `config/noise.yaml` to adjust topic names and noise parameters:

```yaml
/**:
  ros__parameters:
    topics:
      odom_input: "/S550/ground_truth/odom"
      odom_output: "/mavros/local_position/odom"
      pose_output: "/S550/pose"
    noise:
      position_stddev: 0.005
      angle_stddev: 0.005
      axis_stddev: 0.005
      linear_velocity_stddev_x: 0.028
      linear_velocity_stddev_y: 0.022
      linear_velocity_stddev_z: 0.031
      angular_velocity_stddev_x: 0.071
      angular_velocity_stddev_y: 0.054
      angular_velocity_stddev_z: 0.038
      apply_noise: true
      z_offset: 0.0
```

### Parameter Description

| Parameter | Description |
|-----------|-------------|
| `position_stddev` | Standard deviation for position noise (meters) |
| `angle_stddev` | Standard deviation for angle noise (radians) |
| `axis_stddev` | Standard deviation for rotation axis noise |
| `linear_velocity_stddev_{x,y,z}` | Per-axis standard deviation for linear velocity noise (m/s) |
| `angular_velocity_stddev_{x,y,z}` | Per-axis standard deviation for angular velocity noise (rad/s) |
| `apply_noise` | Enable/disable noise injection |
| `z_offset` | Vertical offset from base_link to sensor link (meters) |

## Topics

All topic names are configurable via the yaml file.

### Subscribed Topics
- Input odometry topic (nav_msgs/Odometry): configurable via `topics.odom_input`

### Published Topics
- Noisy odometry topic (nav_msgs/Odometry): configurable via `topics.odom_output`
  - Linear velocity is published in **body frame** ($v_{body} = q^{-1} \cdot v_{world}$)
- Noisy pose topic (geometry_msgs/PoseStamped): configurable via `topics.pose_output`

## Implementation Details

### Z Offset (Rigid-Body Transform)

When `z_offset` is nonzero, the node applies a rigid-body transform from `base_link` to the sensor link:
- Position: $p_{sensor} = p_{base} + q \cdot [0, 0, z_{offset}]^T$
- Velocity: $v_{sensor} = v_{base} + \omega_{world} \times r_{world}$ where $r_{world} = q \cdot [0, 0, z_{offset}]^T$

### Quaternion Noise Injection

1. Convert quaternion to angle-axis representation
2. Apply Gaussian noise to the angle
3. Apply Gaussian noise to each axis component
4. Normalize the axis vector
5. Convert back to quaternion

### Body Frame Velocity

After noise injection, linear velocity is rotated from world frame to body frame:

$$v_{body} = q^{-1} \cdot v_{world}$$

This matches the convention expected by MAVROS (`/mavros/local_position/odom`).
