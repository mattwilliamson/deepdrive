# Deepdrive Vision

## Run litellm docker container
https://litellm.vercel.app/docs/providers/ollama

```sh

docker run -it --net host --name ollama litellm/ollama
docker exec -it ollama ollama pull llama2
docker exec -it ollama ollama pull mixtral
```

## Install Requirements

```sh
make dockersimshell

python3 -m pip install -r src/deepdrive_vision/requirements.txt

ros2 run deepdrive_vision vision_service
```

## Playback Rosbag
https://github.com/ros2/rosbag2
```sh
mkdir src/bag
cd src/bag
ros2 bag record -a
ros2 bag play src/bag/rosbag2_2024_01_22-09_43_40
```

## Foxglove studio
```sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```



## Example Payloads
```shell
ros2 launch deepdrive_camera yolov4_publisher.launch.py
```

Topic: `/color/yolov4_Spatial_detections`

```json
{
  "header": {
    "stamp": {
      "sec": 1705790942,
      "nsec": 771665947
    },
    "frame_id": "oak_rgb_camera_optical_frame"
  },
  "detections": [
    {
      "results": [
        {
          "class_id": "56",
          "score": 0
        }
      ],
      "bbox": {
        "center": {
          "position": {
            "x": 80.5,
            "y": 280
          },
          "theta": 0
        },
        "size_x": 157,
        "size_y": 242
      },
      "position": {
        "x": -0.3271319568157196,
        "y": -0.18792679905891418,
        "z": 1.4815170764923096
      },
      "is_tracking": false,
      "tracking_id": ""
    }
  ]
}
```