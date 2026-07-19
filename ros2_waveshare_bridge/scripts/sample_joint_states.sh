python3 -c "
import rclpy, math
from sensor_msgs.msg import JointState
rclpy.init()
def cb(msg):
    print('\n--- Current Raw Tick Estimates ---')
    for name, rad in zip(msg.name, msg.position):
        # We estimate raw ticks based on whether the joint index is odd or even
        is_even = name in ['shoulder_lift', 'wrist_flex', 'gripper'] # IDs: 2, 4, 6
        ticks = int(2048 - (rad * (2048.0 / math.pi))) if is_even else int(2048 + (rad * (2048.0 / math.pi)))
        print(f'{name:15s}: {ticks:4d} ticks')
    rclpy.shutdown()
node = rclpy.create_node('calibrator')
node.create_subscription(JointState, '/joint_states', cb, 10)
rclpy.spin_once(node)
"