"""
Loop through each axis and each angle and check if the IMU is reading the correct acceleration

Measure +9.81
 meters per second squared for the Z
 axis.
If the sensor is rolled +90
 degrees (left side up), the acceleration should be +9.81
 meters per second squared for the Y
 axis.
If the sensor is pitched +90
 degrees (front side down), it should read -9.81
 meters per second squared for the X
 axis.
"""
# Run the transformer node
# ros2 run imu_transformer imu_transformer_node --ros-args -p target_frame:=imu_link -r imu_in:=/camera_depth/imu/data -r imu_out:=imu 

import os
import signal
import subprocess
import time
import shlex

angs = [0, 1.57, 3.14, -1.57, -3.14]
min_accel = 5.0 # 9.0

def check_axis(axis, valid_angles):
    print("Checking axis: ", axis)
    new_valid_angles = []
    for i, (a1, a2, a3) in enumerate(valid_angles):
        print(f"\nChecking angles: {i+1}/{len(valid_angles)} ({((i+1)/len(valid_angles))*100.0}%): {a1}, {a2}, {a3}")
        # Publish static transform for a moment
        print("Publishing static transform for: ", a1, a2, a3)
        cmd = "bash -c 'ros2 run tf2_ros static_transform_publisher 0 0 0 {:.2f}, {:.2f}, {:.2f} imu_link camera_depth_imu_frame'".format(a1, a2, a3)
        p = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE) 
        time.sleep(2)
        p.kill()
        
        # Get updated IMU measurements
        # Try a couple times in case of noise or delay
        # for _ in range(1):
        imu = subprocess.run(shlex.split("bash -c 'ros2 topic echo imu --once | grep -A 3 linear_acceleration'"), capture_output=True)
        vals = [x.decode("utf-8").strip().split(': ') for x in imu.stdout.split(b'\n')[1:4]]
        vals = dict((x[0], float(x[1])) for x in vals)
        print("Got vals: ", vals)

        if vals[axis] > min_accel:
            new_valid_angles.append([a1, a2, a3])
            print(f"! Got a match ({vals[axis]}): ", a1, a2, a3)
            print("len(new_valid_angles): ", len(new_valid_angles))
            # break
        else:
            print(f"No match ({vals[axis]}): ", a1, a2, a3)
                # time.sleep(1)

    print("new_valid_angles: ", new_valid_angles)
    return new_valid_angles

if __name__ == "__main__":
    print("Starting IMU calibration...")
    # Start the transformer node
    # cmd = "ros2 run imu_transformer imu_transformer_node --ros-args -p target_frame:=imu_link -r imu_in:=/camera_depth/imu/data -r imu_out:=imu"
    # p = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE) 
    # time.sleep(2)
    # print("Started transformer node")

    # Loop through each axis and each angle and check if the IMU is reading the correct acceleration
    # angles = [[x, y, z ] for x in angs for y in angs for z in angs]
    # print(f"- angles {len(angles)} = {angles}")

    # input("Lay the robot flat. Press enter to continue")        
    # angles2 = check_axis('z', angles)
    # print(f"- angles2 {len(angles2)} = {angles2}")

    # input("Roll the robot so the left side is up. Press enter to continue")
    # angles3 = check_axis('y', angles2)
    # print(f"- angles3 {len(angles3)} = {angles3}")

    angles3 = [[1.57, 3.14, -1.57], [1.57, -3.14, -1.57], [-1.57, 0, 1.57]]

    input("Roll the robot so the front side is up. Press enter to continue")
    angles4 = check_axis('x', angles3)
    print(f"- angles4 {len(angles4)} = {angles4}")

    print(angles4)