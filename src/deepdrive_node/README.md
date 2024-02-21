# deepdrive_node

Talks to Raspberry PI pico to control motors and read some sensors.




# TODO: remove ros2_control from diagram

Uses `deepdrive_control/DeepdriveSystemHardware` as the hardware interface. 

This would probably be more efficient not using the micro ros framework or without ros2 control. We'll get some latency going through the topics. Once it's working, we can optimize later. For now, we can probably lower the control loop rates to prevent the topics from being overflowed.

```mermaid
graph TD;
    teleop((Teleop)) --> c1{{cmd_vel}}
    
    subgraph jon  [Jetson Orin Nano]
        c1 --> ros2_control("`**ros2_control**`")
        ros2_control --> diff_drive_controller 
        diff_drive_controller --> dc("`**deepdrive_control**`")
        dc --> c2{{/deepdrive_micro/motor_vel}}

        c2 --> microros[[micro ros agent]]

        ros2_control --> odom2{{/diff_drive_controller/odom}}
        odom2 --> rl(robot_localization)
        rl --> odom{{odom}}
        odom --> nav2
        odom --> slam_toolbox

        microros --> feedback{{/deepdrive_micro/feedback}}
        feedback --> dc

        imu{{/imu}} --> imu_transformer
        imu_transformer --> rl(robot_localization)
        camera_imu{{/camera_depth/imu/data}} --> imu
        
    end
    
    
    subgraph rpipico [Raspberry PI Pico]
        microros -->|Serial USB| rpi(Raspberry PI Pico)
        rpi --> l293(L293 motor controller)
        
        counter(Photointerrupt Pulse Counter) --> rpi
        rpi --> microros
    end
```

Topic `/deepdrive_micro/motor_vel` will take the number of ticks/s desired by the motor controller and which direction and which motor. 

TODO: figure out which message and names to use for `/deepdrive_micro/motor_vel` and `/deepdrive_micro/feedback` 

`deepdrive_control/DeepdriveSystemHardware` subscribes to `/deepdrive_micro/feedback` to get the number of ticks total and number of ticks/s. 