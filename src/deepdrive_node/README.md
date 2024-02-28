# Differential drive

## Installation
```sh
colcon build --symlink-install --packages-select deepdrive_node
source install/setup.sh
```

Make sure to activate your workspace afterwards.

## Usage

```sh
ros2 run deepdrive_node deepdrive_node
ros2 launch deepdrive_node deepdrive_node.launch.py
```

## Tests
```sh
colcon build --symlink-install --packages-select deepdrive_node
colcon test --packages-select deepdrive_node
colcon test-result --verbose
```

TODO:
- [ ] timer for odom output
- [ ] acceleration/jerk
- [ ] Update PID params on the fly
- [ ] Round 0 values
- [ ] Put PID controller on microcontroller. This would make one signal easier to share between wheel. PID would make them match. https://github.com/pms67/PID/tree/master


Talks to Raspberry PI pico via micro ros to control motors and read some sensors.

1 PID controller per wheel
Raspberry Pi sends TOTAL pulse counts to `/pulses`. PID controller uses the diff since the last message to check the velocity.


```mermaid
graph TD;
    teleop((Teleop)) --->|twist| cmd_vel{{/cmd_vel}}
    cmd_vel --> diff_drive

    subgraph "deepdrive_node"
        
        wheel{{wheel}} ----> odometry
    
        diff_drive -->|"[4] Int64"| wheel_vtarget{{pulses/target}}
        wheel ---> pid_velocity
        wheel_vtarget --> pid_velocity
    end

    odometry --> tf{{/tf}}
    odometry --> dd_odom{{/dd/odom}}

    pid_velocity -->|"[4] Int64"| motor{{/motor}}
    motor --> urosa(micro-ros agent)

    subgraph rpipico [Raspberry PI Pico]
        urosa -->|USB Serial| uros(micro-ros)
        
        urosa -->|"[4] Int64"| wheel{{/pulses}}
        
        uros --> l293(L293 controller)

        l293 -->|PWM| dc_motor
        counter(encoder) --> uros
        uros --> urosa
    end
```

---

```mermaid
graph TD;
    subgraph localization

        imu{{/imu}} --> imu_transformer
        imu_transformer --> rl(robot_localization)
        camera_imu{{/camera_depth/imu/data}} --> imu
        oak-d-lite --> camera_imu

        imu2{{/bno08x/imu/data}} --> rl
        imu_bno08x --> imu2
        
        odom --> nav2
        odom --> slam_toolbox

        dd_odom{{/dd/odom}} ---> rl

        rl --> odom{{/odom}}

        tf{{/tf}} ---> nav2
    end    
    
```
