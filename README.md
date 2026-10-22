# ROS2 Bridge for Feetech/Waveshare Servos

## Summary
The following repository illustrates two different approaches of intergrating [feetech servos](https://www.amazon.com/s?k=feetech+servo) into [ROS2](https://www.ros.org). It also contains various tools for working with the feetech servos and diagnosing issues with the feetech driver.

These include:
1. [feetech_ros2_driver](https://github.com/ros-physical-ai/feetech_ros2_driver): This is an open source integration that uses ros2_control as the integration point. In this repository we also expand the telemetry by patching that driver to support the publishing of metrics like current, voltage, temperature, and load.

2. *ros2 bridge*: This approach is a bare-bones ros2 bridge for as waveshare controller and feetech servos. It does not integrate with ros2_control (it is written in python) but follows the same pattern for setting parameters and publishing topcis. Its development was motivated by Certain waveshare controllers for the servos have timing issues with the feetech driver. This is a replacement repository for the feetech_ros2_driver for those of developers who run into bandwidth or timing issues.

These two approaches are mutually exclusive in that you can only run one or the other but not both at the same time.

## Feetech Controllers
TODO: describe the two different types of controllers.

## Features
1. Ability to set the positions of the servos using radians using joint trajectory topic. Compatible with the Dynamixel servos on ros2 control.
2. Ability to collect positions of the servos using the joint state topic. Also compatible with Dynamixel servos and control integration. 
3. Ability to collect advanced servo state that is compatible with the dynamixel servos for temperature, current, voltage, and load. This includes Dynamixel approach to direction (e.g. negaative current and load).

## Contents

The contents of this repository include ros2 launch or related software. In addition there tools which might operate independently of the ros2 framework.

- [`ros2_waveshare_bridge.py`](./ros2_waveshare_bridge/ros2_waveshare_bridge/ros2_waveshare_bridge.py): This is the main controller. It has a few parameters which include a path to your robots urdf file. It subscribes to topics like `JointTrajectory`, `JointState`, and `FeetechState` and then translates these topics into the corresponding Feetech servo commands and passes the results back to the ros2 nodes that issued the topic requests.

- [`bridge.launch.py`](./ros2_waveshare_bridge/launch/bridge.launch.py): This is a sample launch file for the bridge node.

- [`two-servo-arm.urdf`](./ros2_waveshare_bridge/urdf/two-servo-arm.urdf): a simple two servo configuration (in case that is all of the hardware you have).

- [`so-arm101.sample.urdf`](./ros2_waveshare_bridge/urdf/so-arm101.sample.urdf): a more complex urdf for the SO-ARM101 robot. 

- [`waveshare_for_feetch.ino`](./firmware/waveshare_for_feetech.ino): The firmware for the waveshare controller.

- [`set_servo_id.ino`](./firmware/set_servo_id.ino): script to set the servo id. From the factory they are set with 1. 

## 
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

## launch ROS2 with ros2_control and feetech_ros2_driver

```
~/waveshare_jazzy_ws$ source ./install/setup.bash
~/waveshare_jazzy_ws$ ros2 launch ros2_waveshare_bridge feetech_ros2_control.launch.py
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-10-21-23-47-03-367339-bueche-rpi5-7558
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [7561]
[INFO] [ros2_control_node-2]: process started with pid [7562]
[INFO] [spawner-3]: process started with pid [7563]
[robot_state_publisher-1] [INFO] [1792626423.529363666] [robot_state_publisher]: Robot initialized
[ros2_control_node-2] [INFO] [1792626423.548076312] [controller_manager]: Using Steady (Monotonic) clock for triggering controller manager cycles.
[ros2_control_node-2] [INFO] [1792626423.553602720] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[ros2_control_node-2] [INFO] [1792626423.556671575] [controller_manager]: update rate is 50 Hz
[ros2_control_node-2] [INFO] [1792626423.556723556] [controller_manager]: Overruns handling is : enabled
[ros2_control_node-2] [INFO] [1792626423.556738464] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [WARN] [1792626423.556871780] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted). See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
[ros2_control_node-2] [INFO] [1792626423.558418660] [controller_manager]: Received robot description from topic.
[ros2_control_node-2] [INFO] [1792626423.562559059] [controller_manager]: Loading hardware 'FeetechSystem' 
[ros2_control_node-2] [INFO] [1792626423.566873755] [controller_manager]: Loaded hardware 'FeetechSystem' from plugin 'feetech_ros2_driver/FeetechHardwareInterface'
[ros2_control_node-2] [INFO] [1792626423.567043386] [controller_manager]: Initialize hardware 'FeetechSystem' 
[ros2_control_node-2] [2026-10-21 23:47:03.572] [info] Connecting to port: /dev/ttyWaveshare
[ros2_control_node-2] [2026-10-21 23:47:03.576] [info] Loaded joint configuration for 6 joints from '/home/ubuntu/waveshare_jazzy_ws/install/ros2_waveshare_bridge/share/ros2_waveshare_bridge/config/so-arm101.yaml'
[ros2_control_node-2] [2026-10-21 23:47:03.618] [info] Publishing FeetechState on '/feetech_state'
[ros2_control_node-2] [INFO] [1792626423.618340994] [controller_manager]: Successful initialization of hardware 'FeetechSystem'
[ros2_control_node-2] [INFO] [1792626423.619158000] [resource_manager]: 'configure' hardware 'FeetechSystem' 
[ros2_control_node-2] [INFO] [1792626423.619178481] [resource_manager]: Successful 'configure' of hardware 'FeetechSystem'
[ros2_control_node-2] [INFO] [1792626423.619189204] [resource_manager]: 'activate' hardware 'FeetechSystem' 
[ros2_control_node-2] [INFO] [1792626423.621493071] [resource_manager]: Successful 'activate' of hardware 'FeetechSystem'
[ros2_control_node-2] [INFO] [1792626423.621618368] [controller_manager]: Registering statistics for : FeetechSystem
[ros2_control_node-2] [INFO] [1792626423.621808870] [controller_manager]: Resource Manager has been successfully initialized. Starting Controller Manager services...
[ros2_control_node-2] [INFO] [1792626424.126282081] [controller_manager]: Loading controller : 'joint_state_broadcaster' of type 'joint_state_broadcaster/JointStateBroadcaster'
[ros2_control_node-2] [INFO] [1792626424.126338156] [controller_manager]: Loading controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1792626424.131991268] [controller_manager]: Controller 'joint_state_broadcaster' node arguments: --ros-args --params-file /home/ubuntu/waveshare_jazzy_ws/install/ros2_waveshare_bridge/share/ros2_waveshare_bridge/config/controllers.yaml 
[spawner-3] [INFO] [1792626424.163725744] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[ros2_control_node-2] [INFO] [1792626424.164983271] [controller_manager]: Configuring controller: 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1792626424.165234643] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[ros2_control_node-2] [INFO] [1792626424.181271252] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]
[ros2_control_node-2] [INFO] [1792626424.199779230] [controller_manager]: Successfully switched controllers!
[spawner-3] [INFO] [1792626424.222288568] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[ros2_control_node-2] [INFO] [1792626424.223890931] [controller_manager]: Loading controller : 'arm_controller' of type 'joint_trajectory_controller/JointTrajectoryController'
[ros2_control_node-2] [INFO] [1792626424.223951339] [controller_manager]: Loading controller 'arm_controller'
[ros2_control_node-2] [INFO] [1792626424.228220979] [controller_manager]: Controller 'arm_controller' node arguments: --ros-args --params-file /home/ubuntu/waveshare_jazzy_ws/install/ros2_waveshare_bridge/share/ros2_waveshare_bridge/config/controllers.yaml 
[spawner-3] [INFO] [1792626424.261900041] [spawner_joint_state_broadcaster]: Loaded arm_controller
[ros2_control_node-2] [INFO] [1792626424.262323193] [controller_manager]: Configuring controller: 'arm_controller'
[ros2_control_node-2] [INFO] [1792626424.262819511] [arm_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[ros2_control_node-2] [INFO] [1792626424.262924882] [arm_controller]: Command interfaces are [position] and state interfaces are [position velocity].
[ros2_control_node-2] [INFO] [1792626424.263023457] [arm_controller]: Using 'splines' interpolation method.
[ros2_control_node-2] [INFO] [1792626424.265705030] [arm_controller]: Action status changes will be monitored at 20.00 Hz.
[ros2_control_node-2] [INFO] [1792626424.280659540] [controller_manager]: Activating controllers: [ arm_controller ]
[ros2_control_node-2] [INFO] [1792626424.300156024] [controller_manager]: Successfully switched controllers!
[spawner-3] [INFO] [1792626424.322436176] [spawner_joint_state_broadcaster]: Configured and activated arm_controller
```
## launch the bridge

Note: the feetech_ros2_driver ros2_control approach needs to be shutdown before launching this bridge. The two approaches are multually exclusive wrt to their runtimes.

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
    sec: 782673
    nanosec: 561633814
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
- 27
- 29
- 31
- 31
present_input_voltage:
- 121
- 120
- 120
- 121
- 121
- 120
present_current:
- 7
- 46
- 0
- 13
- 7
- 7
present_load:
- -64
- 112
- -20
- 76
- -88
- 56
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

## Running a pose sequence test

```
~/waveshare_jazzy_ws$ ros2 run ros2_waveshare_bridge pose_sequence --ros-args -p pose_file:=src/ros2_waveshare_bridge/config/sample.poses.yaml
[WARN] [1792626632.994642699] [pose_sequence_node]: No joint_config_file provided -- the deviation check will use a coarse default span of 6.28 rad (full single-turn range) for every joint, rather than each joint's real calibrated range. Pass joint_config_file:=<path to your so-arm101.yaml> for a tighter, per-joint tolerance.
[INFO] [1792626633.053228913] [pose_sequence_node]: Pose Sequence Node initialized
[INFO] [1792626633.053912381] [pose_sequence_node]: Loaded 8 poses from src/ros2_waveshare_bridge/config/sample.poses.yaml
[INFO] [1792626633.054687664] [pose_sequence_node]: Joint order: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
[WARN] [1792626633.055398446] [pose_sequence_node]: Make sure these joint names match your URDF/bridge joint names exactly -- the bridge silently ignores any joint name it does not recognize.
[INFO] [1792626633.055856709] [pose_sequence_node]: Starting pose sequence...
[INFO] [1792626633.056384342] [pose_sequence_node]: 
==================================================
[INFO] [1792626633.056864938] [pose_sequence_node]: Moving to pose 1 - poised to work (1/8)
[INFO] [1792626633.057375663] [pose_sequence_node]: Positions: ['-0.054', '-0.011', '0.581', '0.862', '-0.155', '-0.002']
[INFO] [1792626633.058340559] [pose_sequence_node]: Published trajectory, duration: 2s
[INFO] [1792626633.058956156] [pose_sequence_node]: Waiting 2.0s (movement + 0.0s pause)... watching for hardware errors as they happen
[INFO] [1792626636.081359265] [pose_sequence_node]: 
==================================================
[INFO] [1792626636.082189123] [pose_sequence_node]: Moving to pose 2 - reach to grab (2/8)
[INFO] [1792626636.082987202] [pose_sequence_node]: Positions: ['-0.005', '0.006', '-0.209', '0.255', '-0.781', '-0.002']
[INFO] [1792626636.084171914] [pose_sequence_node]: Published trajectory, duration: 2s
[INFO] [1792626636.085073846] [pose_sequence_node]: Waiting 2.0s (movement + 0.0s pause)... watching for hardware errors as they happen
[INFO] [1792626639.120780084] [pose_sequence_node]: 
==================================================
[INFO] [1792626639.121574682] [pose_sequence_node]: Moving to pose 3 - grab (3/8)
[INFO] [1792626639.122348705] [pose_sequence_node]: Positions: ['-0.281', '1.054', '-0.431', '-0.765', '-0.782', '-0.002']
[INFO] [1792626639.123444268] [pose_sequence_node]: Published trajectory, duration: 2s
[INFO] [1792626639.124281626] [pose_sequence_node]: Waiting 2.0s (movement + 0.0s pause)... watching for hardware errors as they happen
[INFO] [1792626642.140815770] [pose_sequence_node]: 
==================================================
[INFO] [1792626642.141425681] [pose_sequence_node]: Moving to pose 4 - lift and center (4/8)
[INFO] [1792626642.142035889] [pose_sequence_node]: Positions: ['-0.282', '1.071', '-0.436', '-0.765', '-0.781', '0.950']
[INFO] [1792626642.142913025] [pose_sequence_node]: Published trajectory, duration: 2s
[INFO] [1792626642.143563029] [pose_sequence_node]: Waiting 2.0s (movement + 0.0s pause)... watching for hardware errors as they happen
[INFO] [1792626645.160844197] [pose_sequence_node]: 
==================================================
[INFO] [1792626645.161659184] [pose_sequence_node]: Moving to pose 5 - raise and get ready to place (5/8)
[INFO] [1792626645.162497875] [pose_sequence_node]: Positions: ['0.143', '1.040', '-0.008', '-1.049', '0.014', '0.552']
[INFO] [1792626645.163665790] [pose_sequence_node]: Published trajectory, duration: 2s
[INFO] [1792626645.164550389] [pose_sequence_node]: Waiting 2.0s (movement + 0.0s pause)... watching for hardware errors as they happen
[INFO] [1792626648.181321960] [pose_sequence_node]: 
==================================================
[INFO] [1792626648.182150262] [pose_sequence_node]: Moving to pose 6 - place high (6/8)
[INFO] [1792626648.182959175] [pose_sequence_node]: Positions: ['0.143', '1.048', '-0.006', '-1.049', '0.017', '-0.014']
[INFO] [1792626648.184149350] [pose_sequence_node]: Published trajectory, duration: 2s
[INFO] [1792626648.185031744] [pose_sequence_node]: Waiting 2.0s (movement + 0.0s pause)... watching for hardware errors as they happen
[INFO] [1792626651.201590129] [pose_sequence_node]: 
==================================================
[INFO] [1792626651.202416190] [pose_sequence_node]: Moving to pose 7 - reach to grab again (7/8)
[INFO] [1792626651.203248196] [pose_sequence_node]: Positions: ['0.357', '-0.216', '-0.155', '0.411', '-1.712', '0.494']
[INFO] [1792626651.204402500] [pose_sequence_node]: Published trajectory, duration: 2s
[INFO] [1792626651.205294691] [pose_sequence_node]: Waiting 2.0s (movement + 0.0s pause)... watching for hardware errors as they happen
[INFO] [1792626654.240865391] [pose_sequence_node]: 
==================================================
[INFO] [1792626654.241690619] [pose_sequence_node]: Moving to pose 8 - poised to work or shutdown (8/8)
[INFO] [1792626654.242482976] [pose_sequence_node]: Positions: ['-0.054', '-0.011', '0.581', '0.862', '-0.155', '-0.002']
[INFO] [1792626654.243606724] [pose_sequence_node]: Published trajectory, duration: 2s
[INFO] [1792626654.244519397] [pose_sequence_node]: Waiting 2.0s (movement + 0.0s pause)... watching for hardware errors as they happen
[INFO] [1792626657.260908873] [pose_sequence_node]: 
==================================================
[INFO] [1792626657.261887139] [pose_sequence_node]: Pose sequence complete!
[INFO] [1792626657.262801627] [pose_sequence_node]: All poses executed successfully.
```
