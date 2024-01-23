# deepdrive_description

This has the URDF files and such to describe how to robot looks and behaves physically.

```sh
make dockersimshell
ros2 launch deepdrive_description display.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```sh
urdf-viz src/deepdrive_description/urdf/deepdrive_deepdrive.urdf
```