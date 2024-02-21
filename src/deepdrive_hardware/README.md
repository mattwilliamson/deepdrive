# Deepdrive Hardware

Talks to Raspberry PI pico via micro ros to control motors and read some sensors.

This is an initial implementation. We'll get some latency going through the topics. Once it's working, we can optimize, do more logic on the microcontroller and do it all in C++ based on the [turtlebot3 repo](https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble-devel/turtlebot3_node). For now, we can probably lower the control loop rates to prevent the topics from being overflowed.

1 PID controller per wheel

```mermaid
graph TD;

    
    teleop((Teleop)) --->|twist| cmd_vel{{/cmd_vel}}
    
    
    diff_tf --> dd_odom{{/dd/odom}}
    diff_tf --> tf{{/tf}}
    cmd_vel --> twist_to_motors
    wheel{{wheel}} ----> diff_tf



    subgraph "wheels x2"
        pid_velocity -->|Float32| motor{{/motor}}
        wheel ---> pid_velocity
        wheel_vtarget --> pid_velocity
        twist_to_motors -->|Float32| wheel_vtarget{{wheel_vtarget}}
        motor --> urosa(micro-ros agent)
    end

    subgraph rpipico [Raspberry PI Pico]
        urosa -->|USB Serial| uros(micro-ros)
        
        urosa -->|Int16| wheel{{/wheel}}
        
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




Pulled from:
https://github.com/eden-desta/ros2_differential_drive/blob/ros2/src/differential_drive

See [differential_drive.md](./differential_drive.md) or https://wiki.ros.org/differential_drive for configuration info.

