# ROS2 Bridge for Waveshare controller and Feetech Servos

## Summary
The following repository has a bare-bones ros2 bridge for the waveshare controller and feetech servos. This is a replacement repository for the [feetech_ros2_driver](https://github.com/ros-physical-ai/feetech_ros2_driver) for those of us who could not get it working (see [here](https://github.com/ros-physical-ai/feetech_ros2_driver/issues/29)). 

It takes a different approach that is perhaps a little less tempermental with the underlying hardware as it is 100% python based (excluding the firmware, that is). This means it will give up a little on latency but should be quite sufficient for most applications (e.g., https://github.com/bueche/ros2_robot_arm).

This will be more thoroughly tested in the next few weeks. Log any issues you find.

## Features
1. Able to set the positions of the servos using radians using joint trajectory topic. Compatible with the Dynamixel servos on ros2 control. Note that ros2_control should ***not** be running.
2. Able to collect positions of the servos using the joint state topic. also compatible with Dynamixel servos and control integration. 
3. Able to collect advanced servo state that is compatible with the dynamixel servos for temperature, current, voltage, and load. This includes Dynamixel approach to direction (e.g. negaative current and load)
4. able to set internal Feetech PID tuning values in a manner that is compatible with the `feetech_ros2_driver`.

## Contents
The contents of this repository include:

- [`ros2_waveshare_bridge.py`](./ros2_waveshare_bridge/ros2_waveshare_bridge/ros2_waveshare_bridge.py): This is the main controller. It has a few parameters which include a path to your robots urdf file. It subscribes to topics like `JointTrajectory`, `JointState`, and `FeetechState` and then translates these topics into the corresponding Feetech servo commands and passes the results back to the ros2 nodes that issued the topic requests.

- [`bridge.launch.py`](./ros2_waveshare_bridge/launch/bridge.launch.py): This is a sample launch file for the bridge node.

- [`two-servo-arm.urdf`](./ros2_waveshare_bridge/urdf/two-servo-arm.urdf): a simple two servo configuration (in case that is all of the hardware you have).

- [`so-arm101.sample.urdf`](./ros2_waveshare_bridge/urdf/so-arm101.sample.urdf): a more complex urdf for the SO-ARM101 robot. 

- [`waveshare_for_feetch.ino`](./firmware/waveshare_for_feetech.ino): The firmware for the waveshare controller.

- [`set_servo_id.ino`](./firmware/set_servo_id.ino): script to set the servo id. From the factory they are set with 1. 

## Installation

```
~/$ mkdir waveshare_jazzy_ws
~/$ cd waveshare_jazzy_ws
~/waveshare_jazzy_ws$ git clone https://github.com/bueche/ros2_bridge_for_feetech_servos.git
Cloning into 'ros2_bridge_for_feetech_servos'...
remote: Enumerating objects: 44, done.
remote: Counting objects: 100% (44/44), done.
remote: Compressing objects: 100% (31/31), done.
remote: Total 44 (delta 9), reused 38 (delta 6), pack-reused 0 (from 0)
Receiving objects: 100% (44/44), 16.16 KiB | 424.00 KiB/s, done.
Resolving deltas: 100% (9/9), done.
~/waveshare_jazzy_ws$ ls
ros2_bridge_for_feetech_servos
~/waveshare_jazzy_ws$ mv ros2_bridge_for_feetech_servos src

```
## build the software
Assumes a ros2 jazzy container

```
~/waveshare_jazzy_ws$ colcon build 
[0.286s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/ubuntu/waveshare_jazzy_ws/install/feetech_interfaces' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.286s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/ubuntu/waveshare_jazzy_ws/install/ros2_waveshare_bridge' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.287s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/ubuntu/waveshare_jazzy_ws/install/feetech_interfaces' in the environment variable CMAKE_PREFIX_PATH doesn't exist
Starting >>> feetech_interfaces
Finished <<< feetech_interfaces [11.1s]                       
Starting >>> ros2_waveshare_bridge
Finished <<< ros2_waveshare_bridge [2.07s]           

Summary: 2 packages finished [13.4s]

```

## launch the bridge

```
~/waveshare_jazzy_ws$ source ./install/setup.bash
~/waveshare_jazzy_ws$ ros2 launch ros2_waveshare_bridge bridge.launch.py
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-06-10-22-00-02-353234-bueche-rpi5-1694
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [bridge_node-1]: process started with pid [1697]
[bridge_node-1] [INFO] [1781128803.016508142] [ros2_waveshare_bridge]: URDF path =: '/home/ubuntu/waveshare_jazzy_ws/install/ros2_waveshare_bridge/share/ros2_waveshare_bridge/urdf/so-arm101.sample.urdf'.
[bridge_node-1] [INFO] [1781128803.017457105] [ros2_waveshare_bridge]: Autonomously Mapped URDF Joint 'shoulder_pan' to Servo ID 1
[bridge_node-1] [INFO] [1781128803.018077327] [ros2_waveshare_bridge]: Autonomously Mapped URDF Joint 'shoulder_lift' to Servo ID 2
[bridge_node-1] [INFO] [1781128803.018655531] [ros2_waveshare_bridge]: Autonomously Mapped URDF Joint 'elbow_flex' to Servo ID 3
[bridge_node-1] [INFO] [1781128803.019208623] [ros2_waveshare_bridge]: Autonomously Mapped URDF Joint 'wrist_flex' to Servo ID 4
[bridge_node-1] [INFO] [1781128803.019782364] [ros2_waveshare_bridge]: Autonomously Mapped URDF Joint 'wrist_roll' to Servo ID 5
[bridge_node-1] [INFO] [1781128803.020340142] [ros2_waveshare_bridge]: Autonomously Mapped URDF Joint 'gripper' to Servo ID 6
[bridge_node-1] [INFO] [1781128803.026064216] [ros2_waveshare_bridge]: Dynamic Bridge Active. Operating 6 joints at 115200 baud.

```

## Simple tests
In another window

```
~/$ cd ~/waveshare_jazzy_ws
~/waveshare_jazzy_ws$ source ./install/setup.bash
~/waveshare_jazzy_ws$ ros2 topic echo /feetech_state --once
header:
  stamp:
    sec: 1781129163
    nanosec: 580917705
  frame_id: ''
comm_state: 0
id:
- 1
- 2
- 3
- 4
- 5
- 6
torque_state:
- true
- true
- true
- true
- true
- true
hw_state:
- 0
- 0
- 0
- 0
- 0
- 0
present_temperature:
- 28
- 28
- 31
- 30
- 29
- 29
present_input_voltage:
- 123
- 124
- 122
- 123
- 124
- 123
present_current:
- 0
- 0
- 0
- 0
- 0
- 0
present_load:
- 0
- 0
- 0
- 0
- 0
- 0
---
~/waveshare_jazzy_ws$ ros2 topic echo /joint_states --once
header:
  stamp:
    sec: 1783374520
    nanosec: 641264517
  frame_id: ''
name:
- shoulder_pan
- shoulder_lift
- elbow_flex
- wrist_flex
- wrist_roll
- gripper
position:
- 1.1780972450961724
- 0.39269908169872414
- 1.5707963267948966
- 1.1780972450961724
- -0.39269908169872414
- -1.1780972450961724
velocity:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
effort:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```

