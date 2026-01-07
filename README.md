# drone_simulator_pkgs

## 1. Install gazebo fortress

Go to the following site, and then, install it.
```
https://gazebosim.org/docs/fortress/install_ubuntu/
```

## 2. Install dependent pkgs

```
sudo apt install ros-humble-ros-gz*
```

Go to the repo and build ros2_libcanard first.
```
https://github.com/kay01-kwon/ros2_libcanard_pkgs
```

ros_motor_model pkg depends on the message type named "ros2_libcanrad_msgs".

## 3. Install this packages.

```
mkdir -p ~/rotor_sim_ws/src
```

```
cd ~/rotor_sim_ws/src
```

```
git clone https://github.com/kay01-kwon/drone_simulator_pkgs.git
```

```
cd ~/rotor_sim_ws/
```

```
colcon build --packages-select drone_description drone_gazebo drone_bringup ros_motor_model --symlink-install
```

## 4. Launch

```
source install/setup.bash
```

```
ros2 launch drone_bringup s550_empty.launch.py
```

## 5. Reset pose of model

```
ign service -s /world/S550_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 5000 --req 'name: "S550" position: {x: 0.0, y: 0.0, z: 1.0} orientation: {w: 1.0, x: 0.0, y:0.0, z:0.0}'
```

## 6. Crieteria for motor time constant (Simulation only)

$ \dot{\omega} = \frac{\omega_{cmd}-\omega}{\tau}$

Maxon motor - maximum acceleration

$ \dot{\omega}_{max}$ = 20,000 rpm/s

10 ms cycle

$\Delta \omega = \omega_{cmd} - \omega= \dot{\omega}_{max} * 0.010 s = 200 rpm$

$\tau = \frac{\Delta \omega}{\dot{\omega}_{max}}=\frac{200 rpm}{20,000 rpm/s} = 0.01 s$