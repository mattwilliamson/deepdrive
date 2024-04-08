
```sh

ros2 launch deepdrive_camera pointcloud.launch.py
```

Config:
https://docs-beta.luxonis.com/software/ros/depthai-ros/driver/#Depthai%20ROS%20Driver-Specific%20camera%20configurations

https://docs.luxonis.com/projects/api/en/latest/tutorials/low-latency/
```
You can also reduce frame latency by using Zero-Copy branch of the DepthAI. This will pass pointers (at XLink level) to cv2.Mat instead of doing memcopy (as it currently does), so performance improvement would depend on the image sizes you are using. (Note: API differs and not all functionality is available as is on the message_zero_copy branch)
```