#!/usr/bin/env python3
"""
ROS2 node to move the SO-ARM101 (via the waveshare bridge) through a sequence
of poses defined in a yaml file. Publishes JointTrajectory messages, pausing
between poses.
"""

import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

TRAJECTORY_TOPIC = '/joint_trajectory_controller/joint_trajectory'  # the waveshare bridge's subscription
DEFAULT_DURATION_SEC = 2      # used when a pose doesn't specify its own 'duration'
POST_MOVE_PAUSE_SEC = 2.0     # matches the original script's fixed 2s pause after each move


class PoseSequenceNode(Node):
    def __init__(self):
        super().__init__('pose_sequence_node')

        self.declare_parameter('pose_file', '')
        pose_file = self.get_parameter('pose_file').get_parameter_value().string_value
        if not pose_file:
            self.get_logger().error(
                "No pose_file provided. Launch with: "
                "ros2 run <pkg> pose_sequence --ros-args -p pose_file:=/path/to/poses.yaml")
            sys.exit(1)

        self.poses, self.joint_names = self.load_poses(pose_file)

        self.publisher = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)

        self.get_logger().info('Pose Sequence Node initialized')
        self.get_logger().info(f'Loaded {len(self.poses)} poses from {pose_file}')
        self.get_logger().info(f'Joint order: {self.joint_names}')
        self.get_logger().warn(
            'Make sure these joint names match your URDF/bridge joint names exactly -- '
            'the bridge silently ignores any joint name it does not recognize.')

    def load_poses(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Could not read/parse pose file '{path}': {e}")
            sys.exit(1)

        raw_poses = data.get('poses') if data else None
        if not raw_poses:
            self.get_logger().error(f"No 'poses' list found in '{path}'")
            sys.exit(1)

        # Joint order comes from the first pose's keys and is then enforced on
        # every subsequent pose, so publish order stays consistent all the way
        # through the run.
        joint_names = list(raw_poses[0]['positions'].keys())

        poses = []
        for raw in raw_poses:
            name = raw.get('name', '(unnamed pose)')
            positions_dict = raw.get('positions', {})
            missing = [j for j in joint_names if j not in positions_dict]
            if missing:
                self.get_logger().error(f"Pose '{name}' is missing joint(s): {missing}")
                sys.exit(1)
            positions = [float(positions_dict[j]) for j in joint_names]
            duration = int(raw.get('duration', DEFAULT_DURATION_SEC))
            poses.append({'name': name, 'positions': positions, 'duration': duration})
        return poses, joint_names

    def create_trajectory_msg(self, positions, duration_sec):
        """Create a JointTrajectory message for the given positions."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)

        msg.points = [point]
        return msg

    def run_sequence(self):
        """Execute the pose sequence with pauses between poses."""
        self.get_logger().info('Starting pose sequence...')

        for i, pose in enumerate(self.poses):
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Moving to {pose['name']} ({i+1}/{len(self.poses)})")
            self.get_logger().info(f"Positions: {[f'{p:.3f}' for p in pose['positions']]}")

            msg = self.create_trajectory_msg(pose['positions'], pose['duration'])
            self.publisher.publish(msg)

            self.get_logger().info(f"Published trajectory, duration: {pose['duration']}s")

            wait_time = pose['duration'] + POST_MOVE_PAUSE_SEC
            self.get_logger().info(f"Waiting {wait_time}s (movement + {POST_MOVE_PAUSE_SEC}s pause)...")
            time.sleep(wait_time)

        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info('Pose sequence complete!')
        self.get_logger().info('All poses executed successfully.')


def main(args=None):
    rclpy.init(args=args)

    node = PoseSequenceNode()

    try:
        node.run_sequence()

        # Keep node alive briefly to ensure last message is sent
        time.sleep(1.0)

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
