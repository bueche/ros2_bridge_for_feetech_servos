#!/usr/bin/env python3
"""
ROS2 node to move the SO-ARM101 (via the waveshare bridge) through a sequence
of poses defined in a yaml file. Publishes JointTrajectory messages, pausing
between poses.
"""

import math
import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from feetech_interfaces.msg import FeetechState

TRAJECTORY_TOPIC = '/joint_trajectory_controller/joint_trajectory'  # the waveshare bridge's subscription
DEFAULT_DURATION_SEC = 2      # used when a pose doesn't specify its own 'duration'
POST_MOVE_PAUSE_SEC = 0.0     # matches the original script's fixed 2s pause after each move
DEFAULT_JOINT_SPAN_RAD = 2 * math.pi  # fallback span if no joint_config_file is given
MONITOR_POLL_SEC = 0.05       # how often to check for hw_state changes during the wait


def decode_error_byte(err_byte):
    # Must stay in sync with the identical decode in ros2_waveshare_bridge.py.
    # bit0=Voltage, bit1=Sensor, bit2=Temperature, bit3=Current, bit4=Angle, bit5=Overload.
    flags = ['Voltage', 'Sensor', 'Temperature', 'Current', 'Angle', 'Overload']
    return [flags[i] for i in range(6) if err_byte & (1 << i)]


class PoseSequenceNode(Node):
    def __init__(self):
        super().__init__('pose_sequence_node')

        self.declare_parameter('pose_file', '')
        self.declare_parameter('joint_config_file', '')  # optional: enables a proper per-joint tolerance
        self.declare_parameter('deviation_threshold_fraction', 0.15)

        pose_file = self.get_parameter('pose_file').get_parameter_value().string_value
        joint_config_file = self.get_parameter('joint_config_file').get_parameter_value().string_value
        self.threshold_fraction = self.get_parameter('deviation_threshold_fraction').get_parameter_value().double_value

        if not pose_file:
            self.get_logger().error(
                "No pose_file provided. Launch with: "
                "ros2 run <pkg> pose_sequence --ros-args -p pose_file:=/path/to/poses.yaml")
            sys.exit(1)

        self.poses, self.joint_names = self.load_poses(pose_file)
        self.joint_span_rad = self.load_joint_spans(joint_config_file, self.joint_names)

        self.publisher = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)

        self.latest_positions = {}
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        self.latest_hw_state = {}  # servo_id -> raw error byte
        self.feetech_sub = self.create_subscription(FeetechState, '/feetech_state', self._on_feetech_state, 10)

        self.get_logger().info('Pose Sequence Node initialized')
        self.get_logger().info(f'Loaded {len(self.poses)} poses from {pose_file}')
        self.get_logger().info(f'Joint order: {self.joint_names}')
        self.get_logger().warn(
            'Make sure these joint names match your URDF/bridge joint names exactly -- '
            'the bridge silently ignores any joint name it does not recognize.')

    def _on_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest_positions[name] = pos

    def _on_feetech_state(self, msg):
        for servo_id, hw_state in zip(msg.id, msg.hw_state):
            self.latest_hw_state[servo_id] = hw_state

    def load_joint_spans(self, path, joint_names):
        if not path:
            self.get_logger().warn(
                "No joint_config_file provided -- the deviation check will use a coarse "
                f"default span of {DEFAULT_JOINT_SPAN_RAD:.2f} rad (full single-turn range) "
                "for every joint, rather than each joint's real calibrated range. Pass "
                "joint_config_file:=<path to your so-arm101.yaml> for a tighter, per-joint "
                "tolerance.")
            return {name: DEFAULT_JOINT_SPAN_RAD for name in joint_names}

        try:
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Could not read/parse joint_config_file '{path}': {e}")
            sys.exit(1)

        joints_cfg = (cfg or {}).get('joints', {})
        spans = {}
        for name in joint_names:
            jcfg = joints_cfg.get(name)
            if not jcfg or 'range_min' not in jcfg or 'range_max' not in jcfg:
                self.get_logger().warn(
                    f"No range_min/range_max for '{name}' in {path}; using default span for it.")
                spans[name] = DEFAULT_JOINT_SPAN_RAD
            else:
                # Tick-to-radian scale (pi/2048) is fixed regardless of drive_mode or
                # homing_offset -- drive_mode only affects which end is "positive", not
                # the physical span size, so this is safe to compute directly from ticks.
                spans[name] = abs(jcfg['range_max'] - jcfg['range_min']) * (math.pi / 2048.0)
        return spans

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

    def monitor_during_move(self, wait_time):
        """Poll /feetech_state throughout the wait (not just once at the end) and
        log the instant an error flag is raised or clears, timestamped relative to
        when this move started -- makes it possible to line up a transient error
        with exactly what the servo was doing at that moment, instead of only
        seeing the end state after the fact."""
        rclpy.spin_once(self, timeout_sec=0.05)  # get whatever's most current right now
        last_logged = dict(self.latest_hw_state)  # baseline: don't re-log pre-existing state

        start = time.time()
        while time.time() - start < wait_time:
            rclpy.spin_once(self, timeout_sec=MONITOR_POLL_SEC)
            elapsed = time.time() - start
            for servo_id, hw_state in self.latest_hw_state.items():
                prev = last_logged.get(servo_id, 0)
                if hw_state != prev:
                    if hw_state > 0:
                        flags = decode_error_byte(hw_state)
                        self.get_logger().error(
                            f"  [t={elapsed:+.2f}s] servo id {servo_id}: hardware error RAISED "
                            f"-- {flags} (raw byte {hw_state:#04x})")
                    elif prev > 0:
                        self.get_logger().info(f"  [t={elapsed:+.2f}s] servo id {servo_id}: hardware error CLEARED")
                    last_logged[servo_id] = hw_state

    def check_pose_reached(self, target_positions):
        """Sample current state and report any joint that's out of tolerance or
        flagging a real hardware error. Returns a list of problem strings (empty
        list = all clear)."""
        # Drain the subscription queues for a full second so latest_positions/
        # latest_hw_state reflect a fresh reading from the bridge, not a stale one
        # from before this move (the bridge's telemetry timer publishes on its own
        # cycle, not on request).
        deadline = time.time() + 1.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        problems = []
        for name, target in zip(self.joint_names, target_positions):
            actual = self.latest_positions.get(name)
            if actual is None:
                problems.append(f"{name}: no /joint_states data received yet")
                continue
            tolerance = self.threshold_fraction * self.joint_span_rad[name]
            deviation = abs(actual - target)
            if deviation > tolerance:
                problems.append(
                    f"{name}: target={target:+.3f} rad, actual={actual:+.3f} rad, "
                    f"deviation={deviation:.3f} rad exceeds tolerance {tolerance:.3f} rad "
                    f"({self.threshold_fraction*100:.0f}% of {self.joint_span_rad[name]:.3f} rad span)")

        for servo_id, hw_state in self.latest_hw_state.items():
            if hw_state > 0:
                flags = decode_error_byte(hw_state)
                problems.append(f"servo id {servo_id}: hardware error flag(s) set -- {flags} (raw byte {hw_state:#04x})")
            elif hw_state < 0:
                problems.append(f"servo id {servo_id}: hw_state not yet read this cycle -- comms may be unreliable")

        return problems

    def run_sequence(self):
        """Execute the pose sequence with pauses between poses. Stops immediately
        if a pose isn't reached within tolerance or a servo reports a hardware
        error."""
        self.get_logger().info('Starting pose sequence...')

        for i, pose in enumerate(self.poses):
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Moving to {pose['name']} ({i+1}/{len(self.poses)})")
            self.get_logger().info(f"Positions: {[f'{p:.3f}' for p in pose['positions']]}")

            msg = self.create_trajectory_msg(pose['positions'], pose['duration'])
            self.publisher.publish(msg)

            self.get_logger().info(f"Published trajectory, duration: {pose['duration']}s")

            wait_time = pose['duration'] + POST_MOVE_PAUSE_SEC
            self.get_logger().info(f"Waiting {wait_time}s (movement + {POST_MOVE_PAUSE_SEC}s pause)... watching for hardware errors as they happen")
            self.monitor_during_move(wait_time)

            problems = self.check_pose_reached(pose['positions'])
            if problems:
                self.get_logger().error(f"Pose '{pose['name']}' failed validation -- stopping sequence:")
                for p in problems:
                    self.get_logger().error(f"  - {p}")
                return False

        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info('Pose sequence complete!')
        self.get_logger().info('All poses executed successfully.')
        return True


def main(args=None):
    rclpy.init(args=args)

    node = PoseSequenceNode()
    success = False

    try:
        success = node.run_sequence()

        # Keep node alive briefly to ensure last message is sent
        time.sleep(1.0)

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
