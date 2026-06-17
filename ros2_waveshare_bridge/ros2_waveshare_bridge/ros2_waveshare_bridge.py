#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from feetech_interfaces.msg import FeetechState
import serial
import threading
import time
import math
import os
import xml.etree.ElementTree as ET

class Ros2WaveshareBridge(Node):
    def __init__(self):
        super().__init__('ros2_waveshare_bridge')
        
        # Declare Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('urdf_path', '') # Absolute path to your URDF or XACRO file
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        
        self.joint_map = {}
        self.joint_names = []
        self.active_ids = []

        # Autonomously Parse URDF Target Layout
        if urdf_path and os.path.exists(urdf_path):
            self.parse_urdf_file(urdf_path)
        else:
            self.get_logger().error(f"URDF path missing or invalid: '{urdf_path}'. Waiting for configuration.")
            return

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.03, rtscts=False, dsrdtr=False)
            self.get_logger().info(f"Dynamic Bridge Active. Operating {len(self.joint_names)} joints at {baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open port: {e}")
            return

        self.serial_lock = threading.Lock()

        # Publishers & Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.trajectory_callback, 10
        )
        self.feetech_state_pub = self.create_publisher(FeetechState, '/feetech_state', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.telemetry_timer = self.create_timer(0.25, self.query_telemetry_callback)

    def parse_urdf_file(self, path):
        """ Parses a local URDF file completely automatically to extract joints and embedded IDs """
        try:
            tree = ET.parse(path)
            root = tree.getroot()
            
            for joint in root.findall('joint'):
                j_name = joint.get('name')
                j_type = joint.get('type')
                
                # Only target active moving joints
                if j_type in ['revolute', 'continuous']:
                    servo_id = None
                    
                    # Search inside the joint for custom ros2_control hardware configurations
                    r2c = joint.find('ros2_control')
                    if r2c is not None:
                        for param in r2c.findall('param'):
                            if param.get('name') == 'servo_id':
                                servo_id = int(param.text.strip())
                                break
                    
                    # Fallback to look everywhere inside the joint tag elements
                    if servo_id is None:
                        for elem in joint.iter('param'):
                            if elem.get('name') == 'servo_id':
                                servo_id = int(elem.text.strip())
                                break
                                
                    if servo_id is not None:
                        self.joint_map[j_name] = servo_id
                        self.joint_names.append(j_name)
                        self.active_ids.append(servo_id)
                        self.get_logger().info(f"Autonomously Mapped URDF Joint '{j_name}' to Servo ID {servo_id}")
            
            if not self.joint_map:
                self.get_logger().warn("URDF parsed successfully, but no joints with a <param name='servo_id'> were discovered.")
                
        except Exception as e:
            self.get_logger().error(f"Failed to automatically parse URDF file: {e}")

    def calculate_checksum(self, packet_bytes):
        return (~sum(packet_bytes[2:])) & 0xFF

    def read_exact_bytes(self, num_bytes):
        buffer = bytearray()
        start_time = time.time()
        while len(buffer) < num_bytes and (time.time() - start_time) < 0.04:
            chunk = self.ser.read(num_bytes - len(buffer))
            if chunk:
                buffer.extend(chunk)
        return buffer if len(buffer) == num_bytes else buffer

    def radians_to_ticks(self, rad, servo_id):
        if servo_id % 2 == 0:
            ticks = int(2048 - (rad * (2048.0 / math.pi)))
        else:
            ticks = int(2048 + (rad * (2048.0 / math.pi)))
        return max(min(ticks, 4095), 0)

    def waveshare_ticks_to_radians(self, ticks, servo_id):
        if servo_id % 2 == 0:
            return (2048 - ticks) * (math.pi / 2048.0)
        return (ticks - 2048) * (math.pi / 2048.0)

    def trajectory_callback(self, msg):
        if not msg.points:
            return
        target_point = msg.points[-1]
        
        with self.serial_lock:
            for index, joint_name in enumerate(msg.joint_names):
                if joint_name in self.joint_map:
                    servo_id = self.joint_map[joint_name]
                    pos_ticks = self.radians_to_ticks(target_point.positions[index], servo_id)
                    
                    val_h = (pos_ticks >> 8) & 0xFF
                    val_l = pos_ticks & 0xFF
                    
                    packet = bytearray([0xFF, 0xFF, servo_id, 0x05, 0x03, 42, val_h, val_l])
                    packet.append(self.calculate_checksum(packet))
                    try:
                        self.ser.write(packet)
                    except Exception as e:
                        self.get_logger().error(f"Write error: {e}")

    def query_telemetry_callback(self):
        if not self.joint_names:
            return
        now = self.get_clock().now().to_msg()
        
        feetech_msg = FeetechState()
        feetech_msg.header.stamp = now
        feetech_msg.comm_state = FeetechState.COMM_STATE_OK
        feetech_msg.id = self.active_ids
        feetech_msg.torque_state = [True] * len(self.active_ids)
        feetech_msg.hw_state = [0] * len(self.active_ids)

        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = self.joint_names

        positions_rad = []
        temps, volts, currents, loads = [], [], [], []
        success_count = 0

        with self.serial_lock:
            for servo_id in self.active_ids:
                packet = bytearray([0xFF, 0xFF, servo_id, 0x04, 0x02, 56, 15])
                packet.append(self.calculate_checksum(packet))
                
                try:
                    self.ser.reset_input_buffer()
                    self.ser.write(packet)
                    
                    resp = self.read_exact_bytes(21)
                    if resp and len(resp) == 21 and resp[0] == 0xFF and resp[1] == 0xFF:
                        ticks = (resp[5] << 8) | resp[6]
                        load = (resp[9] << 8) | resp[10]
                        volt = resp[11]
                        temp = resp[12]
                        current = (resp[18] << 8) | resp[19]
                        
                        if current > 32767: current -= 65536
                        if load > 32767: load -= 65536
                        
                        positions_rad.append(self.waveshare_ticks_to_radians(ticks, servo_id))
                        temps.append(temp)
                        volts.append(volt)
                        currents.append(current)
                        loads.append(int(load / 10))
                        success_count += 1
                    else:
                        feetech_msg.comm_state = FeetechState.COMM_STATE_ITEM_READ_FAIL
                        positions_rad.append(0.0)
                        temps.append(0); volts.append(0); currents.append(0); loads.append(0)
                except Exception:
                    feetech_msg.comm_state = FeetechState.COMM_STATE_ITEM_READ_FAIL
                    positions_rad.append(0.0)
                    temps.append(0); volts.append(0); currents.append(0); loads.append(0)

        if success_count == len(self.active_ids):
            joint_msg.position = positions_rad
            joint_msg.velocity = [0.0] * len(self.joint_names)
            joint_msg.effort = [0.0] * len(self.joint_names)
            self.joint_state_pub.publish(joint_msg)

        feetech_msg.present_temperature = temps
        feetech_msg.present_input_voltage = volts
        feetech_msg.present_current = currents
        feetech_msg.present_load = loads
        self.feetech_state_pub.publish(feetech_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Ros2WaveshareBridge()
    if hasattr(node, 'ser'):
        rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()