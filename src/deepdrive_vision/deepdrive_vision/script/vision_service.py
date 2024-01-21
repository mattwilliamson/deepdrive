#!/usr/bin/env python


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from sensor_msgs.msg import Image  # Image is the message type
import tempfile



import os

IS_LOCAL = os.getenv("IS_LOCAL", False)
if not IS_LOCAL:
    from deepdrive_vision.deepdrive_vision.vision import VisionDriverL293

# TODO: Probably want a PID controller here

# TODO: Smooth the acceleration

wheelbase = 0.10
wheel_radius = 0.02
KF_TURN = 0.5  # 0.5
KP_TURN = 0.1
KI_TURN = 0.05  # 0.0001

KF_FWD = 0.2  # vel
KP_FWD = 0.2
KI_FWD = 0.1  # 0.0001


class VisionService(Node):
    def __init__(self):
        super().__init__("vision_service")
        self.get_logger().info("vision_service node has been initialised.")

        # We just want to process 1 image at a time as close to realtime as possible
        qos = QoSProfile(depth=1)

        # /color/yolov4_Spatial_detections
        # depthai_ros_msgs/msg/SpatialDetectionArray @ 1705790925.999000000 sec

        self.img_sub = self.create_subscription(
            Image, "video_frames", self.got_image, qos
        )

        # self.odom_publisher = self.create_publisher(Odometry, 'wheel/odometry', 30)

        self.bridge = CvBridge()

    def got_image(self, msg):
        self.get_logger().info('Received an image!')
        # TODO: Don't save until we have a service call
        new_file, filename = tempfile.mkstemp()
        # Convert your ROS Image message to OpenCV2
        # cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = self.bridge.imgmsg_to_cv2(msg)
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite("camera_image.jpeg", cv2_img)
        self.get_logger().info(f'Saving to {filename}')
        os.close(new_file)

    def on_shutdown(self):
        print("vision_service shutting down")


def main(args=None):
    print("Hi from vision_service.")
    rclpy.init(args=args)
    vision_service = VisionService()
    rclpy.spin(vision_service)

    vision_service.destroy_node()
    rclpy.shutdown()
    vision_service.on_shutdown()


if __name__ == "__main__":
    main()
