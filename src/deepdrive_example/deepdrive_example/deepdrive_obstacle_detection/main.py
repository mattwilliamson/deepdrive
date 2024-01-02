#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import rclpy

from deepdrive_example.deepdrive_obstacle_detection.deepdrive_obstacle_detection \
    import DeepdriveObstacleDetection


def main(args=None):
    rclpy.init(args=args)
    deepdrive_obstacle_detection = DeepdriveObstacleDetection()
    rclpy.spin(deepdrive_obstacle_detection)

    deepdrive_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()