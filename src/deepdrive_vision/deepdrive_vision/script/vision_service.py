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
import time
import litellm
import base64
import textwrap

litellm.set_verbose = True

# https://docs.foxglove.dev/docs/visualization/message-schemas/text-annotation/
from foxglove_msgs.msg import (
    TextAnnotation,
    TextMarker,
    Color,
    ImageAnnotations,
    Point2,
    CircleAnnotation,
)


# import openai

# https://medium.com/@dimas230220020/introduce-how-to-using-llava-large-language-and-vision-assistant-439a1aacfdb2
# import threading
# import subprocess
# threading.Thread(target=lambda: subprocess.run(['python3', '-m', 'llava.serve.controller', '--host', '0.0.0.0', '--port', '10000'], check=True), daemon=True).start()

# import threading
# import subprocess
# command = [
#     'python3', '-m', 'llava.serve.model_worker',
#     '--host', '0.0.0.0',
#     '--controller', 'http://localhost:10000',
#     '--port', '40000',
#     '--worker', 'http://localhost:40000',
#     '--model-path', '4bit/llava-v1.5-13b-3GB',
#     '--load-4bit'
# ]
# threading.Thread(target=lambda: subprocess.run(command, check=True, shell=False), daemon=True).start()

# https://microsoft.github.io/autogen/blog/2023/11/06/LMM-Agent

class VisionService(Node):
    def __init__(self):
        super().__init__("vision_service")
        self.get_logger().info("vision_service node has been initialised.")

        # We just want to process 1 image at a time as close to realtime as possible
        qos = QoSProfile(depth=1)

        # /color/yolov4_Spatial_detections
        # depthai_ros_msgs/msg/SpatialDetectionArray @ 1705790925.999000000 sec

        self.img_sub = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.got_image, qos
        )

        self.pub_annotations = self.create_publisher(
            ImageAnnotations, "/oak/rgb/annotations", qos
        )

        self.pub_markers = self.create_publisher(TextMarker, "/oak/rgb/markers", qos)

        self.previous_items = []

        # self.odom_publisher = self.create_publisher(Odometry, 'wheel/odometry', 30)

        self.bridge = CvBridge()
        self.frames = 0

    def got_image(self, msg):
        print('-'*80)
        self.get_logger().info("Received an image!")
        self.frames += 1
        # TODO: Don't save until we have a service call
        # new_file, filename = tempfile.mkstemp()
        # Convert your ROS Image message to OpenCV2
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # self.get_logger().info(f"got image: {cv2_img.shape}")
        # cv2_img = self.bridge.imgmsg_to_cv2(msg)
        # Save your OpenCV2 image as a jpeg
        # cv2.imwrite("camera_image.jpeg", cv2_img)
        # self.get_logger().info(f'Saving to {filename}')
        # os.close(new_file)
        self.get_logger().info("running prompt")
        # p = f"""Which room is this image in? Only answer a name, like 'kitchen' or 'bedroom', not a number. Say 'unknown' if you don't know."""

        p = "What do you see in detail in this image? Which room might it be in?"
        room_detail = self.prompt(p, cv2_img)
        print(f"room_detail: {room_detail}")

        p2 = f"""Which room of the house does this seem like? Only answer a single word like 'living room' or 'kitchen'. Only answer if you have a high confidence, otherwise answer 'unknown'.\nDescription: {room_detail}"""
        room = self.prompt(p2).strip().capitalize()
        print(f"room: {room}")
        # user_prompt = "USER: which room was this picture taken in?"
        # p = f"""{user_prompt} ASSISTANT: <room type> {user_prompt} """
        # room = self.prompt(p, cv2_img)
        # print(f"room_detail: {room}")

        text_out = "AI Predicted Room: " + room
        formatted = textwrap.fill(text_out.strip(), 75)
        # self.get_logger().info(f"got result: (type: {type(text_out)}) {text_out}")
        # self.get_logger().info(f"timestamp: (type: {type(msg.header.stamp)}) {msg.header.stamp} {dir(msg.header.stamp)}")
        text_colors = [Color(r=0.5, g=0.0, b=0.0, a=1.0), Color(r=0.1, g=0.1, b=0.1, a=1.0), Color(r=0.0, g=0.5, b=0.0, a=1.0), Color(r=0.0, g=0.0, b=0.5, a=1.0), Color(r=0.5, g=0.0, b=0.5, a=1.0)]
        ann = TextAnnotation(
            timestamp=msg.header.stamp,
            # position=Point2(x=10.0, y=1024.0),
            position=Point2(x=10.0, y=50.0),
            text=formatted,
            font_size=28.0,
            # text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
            text_color=text_colors[hash(room) % len(text_colors)],
            background_color=Color(r=1.0, g=1.0, b=1.0, a=0.65),
        )

        circle_color = Color(r=1.0, g=0.0, b=0.0, a=1.0) if self.frames % 2 == 0 else Color(r=0.0, g=1.0, b=0.0, a=1.0)
        ann2 = CircleAnnotation(
            timestamp=msg.header.stamp,
            position=Point2(x=-50.0, y=50.0),
            diameter=50.0,
            fill_color=circle_color,
            outline_color=Color(r=0.0, g=0.0, b=0.0, a=1.0),
            thickness=1.0,
        )

        anns = ImageAnnotations(
            # timestamp=msg.header.stamp,
            texts=[ann],
            circles=[ann2],
        )
        self.pub_annotations.publish(anns)

        # marker = TextMarker(
        #     # timestamp=msg.header.stamp,
        #     billboard=True,
        #     font_size=32.0,
        #     color=Color(r=1.0, g=0.0, b=0.0, a=1.0),
        #     scale_invariant=True,
        #     text=text_out,
        # )
        # self.pub_markers.publish(marker)

        time.sleep(1)

    def prompt(self, prompt, image=None):
        # response = openai.chat.completions.create(
        #     model="ollama/llama2",
        #     messages = [
        #         {
        #             "role": "user",
        #             "content": "this is a test request, acknowledge that you got it"
        #         }
        #     ],
        #     stream=True
        # )

        if image is not None:
            retval, buffer = cv2.imencode(".jpg", image)
            payload = base64.b64encode(buffer).decode("ascii")
            # self.get_logger().info(f"base64: {payload[:100]}")

            # if self.previous_items:
            # p = p + "\n\n" + "You previously may have seen these: " + ", ".join(self.previous_items)

            # List some objects in the room that might help identify it in a simple list format. Only list objects that are in the image. Example: kitchen - fridge, stove, sink, people cooking

            # print(f"prompt: {p}")

            response = litellm.completion(
                # model="ollama/bakllava",
                model="ollama/llava",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            # {"type": "text", "text": "Whats in this image?"},
                            # {"type": "text", "text": "Do you see any poeple? How many? Male? Female? How old? What emotions are they expressing?"},
                            {"type": "text", "text": prompt},
                            {"type": "image_url", "image_url": {"url": payload}},
                        ],
                    }
                ],
            )

            # try:
            #     # print(f"# response: {response}")
            #     text = response.choices[0].message.content
            #     print(f"# RESPONSE: {text}")
            #     prev = text.split('-')[1].strip()
            #     self.previous_items.append(prev)
            # except Exception as e:
            #     # self.get_logger().error(f"error: {e}")
            #     # print(f"error: {e}")
            #     # prev = "unknown"
            #     pass
            # if len(self.previous_items) > 5:
            #     self.previous_items.pop(0)
            #     print(f"previous_items: {self.previous_items}")
        else:
            response = litellm.completion(
                # model="ollama/mixtral",
                model="ollama/llama2",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            prompt,
                        ],
                    }
                ],
            )

        return response.choices[0].message.content

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
