# Stereo Camera
Luxonis Oak D Lite

```sh
ros2 launch deepdrive_camera pointcloud.launch.py
```

Config:
https://docs-beta.luxonis.com/software/ros/depthai-ros/driver/#Depthai%20ROS%20Driver-Specific%20camera%20configurations

https://docs.luxonis.com/projects/api/en/latest/tutorials/low-latency/
```
You can also reduce frame latency by using Zero-Copy branch of the DepthAI. This will pass pointers (at XLink level) to cv2.Mat instead of doing memcopy (as it currently does), so performance improvement would depend on the image sizes you are using. (Note: API differs and not all functionality is available as is on the message_zero_copy branch)
```

# Wide Angle Camera
Need IMX219 camera for Jetson Orin Nano
8 megapixel IMX219, Max. resolution: 3280 (H) x 2464 (V)
Interface: 2-Lane MIPI

The Raspberry Pi v2.1 camera uses a 15 pin connector.

FFC/FPC cable is same as Raspberry PI Zero: 15 Pin 1.0mm to 22 Pin 0.5mm Flex Ribbon Extension Cable for Raspberry Pi Zero
This camera cable is specifically designed for Pi camera module and Pi Zero - Version 1.3

https://www.amazon.com/Arducam-Raspberry-Camera-Module-Megapixel/dp/B083BHJZ16

## TODO:
Use Isaac camera node to get 10x performance
https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_argus_camera/index.html


## Calibration
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
https://docs.ros.org/en/rolling/p/image_pipeline/camera_info.html
https://docs.ros.org/en/rolling/p/camera_calibration/index.html


```sh
sudo apt-get install -y v4l-utils

matt@deepdrive:~$ v4l2-ctl --list-devices
NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca):
	/dev/media0

vi-output, imx219 10-0010 (platform:tegra-capture-vi:2):
	/dev/video0
```

## Restart service
```sh
sudo service nvargus-daemon restart
```

Check codecs: `gst-inspect-1.0`

### Nvidia Gstreamer docs: 
https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_plugin_Intro.html

https://github.com/ros-drivers/gscam/blob/ros2/examples/v4l.launch.xml


```sh
ros2 launch deepdrive_camera wide_angle_camera.launch.xml
ros2 launch deepdrive_camera camera.launch.xml

ros2 bag record -a -o src/bags/cameras-2024-05-23
```

## Calibration
https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_VIII:_Start_the_Camera_Node

## Video Webserver
https://github.com/RobotWebTools/web_video_server/tree/ros2