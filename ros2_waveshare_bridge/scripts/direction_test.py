#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Small enough to be safe from the zero pose, large enough to clearly see.
DELTA_RAD = 0.20

# Order matters -- this is the order joints get tested in.
# "expect" is a HYPOTHESIS, not a confirmed fact -- it's our best guess at what a
# small POSITIVE radian command should look like under normal convention. Watch
# the arm and judge for yourself whether it matches; that's the actual test.
# Edit these labels if you disagree with the guess before running.
JOINTS = [
    {"name": "shoulder_pan",  "expect": "LEFT turn (base rotates counter-clockwise, viewed from above)"},
    {"name": "shoulder_lift", "expect": "UP (upper arm rises)"},
    {"name": "elbow_flex",    "expect": "UP (forearm rises relative to upper arm)"},
    {"name": "wrist_flex",    "expect": "UP (hand rises relative to forearm)"},
    {"name": "wrist_roll",    "expect": "RIGHT turn (clockwise, viewed from behind the gripper looking toward the wrist)"},
    {"name": "gripper",       "expect": "OPEN (jaws separate -- it's already fully closed, so this is the only direction that will show anything)"},
]

TRAJECTORY_TOPIC = '/joint_trajectory_controller/joint_trajectory'
JOINT_NAMES = [j["name"] for j in JOINTS]


class DirectionTestNode(Node):
    def __init__(self):
        super().__init__('direction_test')
        self.pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self.latest_positions = {}
        self.sub = self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest_positions[name] = pos

    def wait_for_joint_states(self, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self.latest_positions for name in JOINT_NAMES):
                return True
        return False

    def send_positions(self, positions_by_name):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [positions_by_name[name] for name in JOINT_NAMES]
        msg.points = [point]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DirectionTestNode()

    print("Waiting for /joint_states...")
    if not node.wait_for_joint_states():
        print("ERROR: never received a full /joint_states message. Is the bridge running?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Freeze the starting pose -- every test moves exactly one joint away from this
    # and back, so nothing drifts across the run.
    start_positions = dict(node.latest_positions)
    print("Starting pose captured:")
    for name in JOINT_NAMES:
        print(f"  {name:15s}: {start_positions[name]:+.4f} rad")

    print("\n" + "=" * 60)
    print("           PER-JOINT DIRECTION TEST")
    print("=" * 60)
    print(f"Each joint gets a {DELTA_RAD:+.2f} rad nudge, one at a time, then returns")
    print("to its starting position before the next joint is tested.\n")

    for joint in JOINTS:
        name = joint["name"]
        print(f">>> Testing {name.upper()}")
        print(f"    Expecting: {joint['expect']}")

        nudged = dict(start_positions)
        nudged[name] = start_positions[name] + DELTA_RAD
        node.send_positions(nudged)

        input("    Press [ENTER] once you've observed the direction (this will restore "
              "the joint before testing the next one)...")

        node.send_positions(start_positions)
        time.sleep(0.5)  # let it settle back before moving on

    print("\n" + "=" * 60)
    print("Done. Compare what you saw against each 'Expecting' line above to decide")
    print("drive_mode for each joint: if it moved as expected, drive_mode: 0 is correct;")
    print("if it moved the opposite way, set drive_mode: 1 for that joint in the yaml.")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
