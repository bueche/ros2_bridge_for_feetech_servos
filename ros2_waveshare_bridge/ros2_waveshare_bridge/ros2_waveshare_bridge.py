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
import yaml
import xml.etree.ElementTree as ET

class Ros2WaveshareBridge(Node):
    def __init__(self):
        super().__init__('ros2_waveshare_bridge')
        
        # Declare Parameters
        self.declare_parameter('port', '/dev/ttyIMU') # local should be soft link to USB0 or USB1..etc
        self.declare_parameter('baud', 115200)
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('joint_config_file', '') # Path to the optional YAML configuration file
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        yaml_path = self.get_parameter('joint_config_file').get_parameter_value().string_value
        
        self.joint_map = {}
        self.joint_names = []
        self.active_ids = []
        self.servo_configs = {} # Structured storage for runtime calibration parameters

        # 1. Parse URDF base configuration rules
        if urdf_path and os.path.exists(urdf_path):
            self.parse_urdf_file(urdf_path)
        else:
            self.get_logger().error(f"URDF path missing or invalid: '{urdf_path}'. Waiting for configuration.")
            return

        # 2. Layer YAML modifications over the top if available
        if yaml_path and os.path.exists(yaml_path):
            self.parse_yaml_file(yaml_path)

        # Connect to Hardware Link
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.03, rtscts=False, dsrdtr=False)
            self.get_logger().info(f"Dynamic Bridge Active. Operating {len(self.joint_names)} joints via {port} at {baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open port {port}: {e}")
            return

        self.serial_lock = threading.Lock()

        # 3. Flash runtime calibration parameters to active servos on boot
        self.apply_servo_calibrations()

        # Publishers & Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.trajectory_callback, 10
        )
        self.feetech_state_pub = self.create_publisher(FeetechState, '/feetech_state', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.telemetry_timer = self.create_timer(0.25, self.query_telemetry_callback)


    def parse_urdf_file(self, path):
        """ Robust parser that matches both plain nested-tag URDF layouts and standard ros2_control parent containers """
        try:
            tree = ET.parse(path)
            root = tree.getroot()
            
            # Find every single joint element anywhere in the document
            for joint in root.findall('.//joint') + root.findall('joint'):
                j_name = joint.get('name')
                j_type = joint.get('type')
                
                # Filter out none types, but keep active or unspecified control joints
                if not j_name or j_type == 'fixed':
                    continue
                    
                clean_name = j_name.replace('${prefix}', '').replace('_joint', '')
                servo_id = None
                params = {}
                
                # Gather all parameters inside this joint element or any nested sub-elements
                # This catches both <param> directly in <joint> and <param> inside a nested <ros2_control>
                for param in joint.findall('.//param') + joint.findall('param'):
                    p_name = param.get('name')
                    p_val = param.text.strip() if param.text else ""
                    if p_name:
                        params[p_name] = p_val
                
                # Check for ID under either common naming convention
                if 'servo_id' in params:
                    servo_id = int(params['servo_id'])
                elif 'id' in params:
                    servo_id = int(params['id'])
                    
                if servo_id is not None:
                    self.joint_map[clean_name] = servo_id
                    if clean_name not in self.joint_names:
                        self.joint_names.append(clean_name)
                        self.active_ids.append(servo_id)
                    
                    # Compute baseline defaults from either URDF variant
                    offset_val = int(params.get('offset', 2048))
                    homing_offset = int(params.get('homing_offset', offset_val - 2048))
                    
                    self.servo_configs[servo_id] = {
                        'homing_offset': homing_offset,
                        'p_coefficient': int(params.get('p_coefficient', params.get('p_cofficient', 16))),
                        'i_coefficient': int(params.get('i_coefficient', 0)),
                        'd_coefficient': int(params.get('d_coefficient', 32))
                    }
                    self.get_logger().info(f"Autonomously Mapped Joint '{clean_name}' to Servo ID {servo_id}")
            
            if not self.joint_map:
                self.get_logger().warn("URDF parsed, but no joints containing an 'id' or 'servo_id' parameter were discovered.")
                
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF elements: {e}")
    def parse_yaml_file(self, path):
        """ Merges ecosystem parameters directly out of standard LeRobot / Feetech driver YAML files """
        try:
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
                
            if 'joints' in config:
                for j_name, data in config['joints'].items():
                    clean_name = j_name.replace('_joint', '')
                    if clean_name in self.joint_map:
                        servo_id = self.joint_map[clean_name]
                        
                        # Read configuration keys into tracking dictionary structures
                        for key in ['homing_offset', 'p_coefficient', 'i_coefficient', 'd_coefficient', 
                                    'acceleration', 'return_delay_time', 'max_torque_limit']:
                            if key in data:
                                self.servo_configs[servo_id][key] = int(data[key])
                                
                        self.get_logger().info(f"Merged YAML adjustments over Joint '{clean_name}' (ID: {servo_id})")
        except Exception as e:
            self.get_logger().error(f"Failed to process YAML file configuration adjustments: {e}")

    def write_register(self, servo_id, reg_address, value, num_bytes=1):
        """ Helper utility to format and transmit standard WRITE packets down the serial lines """
        packet = bytearray([0xFF, 0xFF, servo_id])
        length = 4 + num_bytes
        packet.append(length)
        packet.append(0x03) # WRITE Command
        packet.append(reg_address)
        
        if num_bytes == 1:
            packet.append(value & 0xFF)
        else:
            packet.append((value >> 8) & 0xFF)
            packet.append(value & 0xFF)
            
        packet.append((~sum(packet[2:])) & 0xFF)
        self.ser.write(packet)
        time.sleep(0.002)

    def apply_servo_calibrations(self):
        """ Flashes calibration settings sequentially to the hardware registers on boot """
        with self.serial_lock:
            self.get_logger().info("Initializing hardware calibration parameters...")
            for servo_id, cfg in self.servo_configs.items():
                try:
                    # Unlock EEPROM (Register 55 = 0)
                    self.write_register(servo_id, 55, 0, 1)
                    
                    # Write PID Metrics
                    if 'p_coefficient' in cfg: self.write_register(servo_id, 21, cfg['p_coefficient'], 1)
                    if 'i_coefficient' in cfg: self.write_register(servo_id, 22, cfg['i_coefficient'], 1)
                    if 'd_coefficient' in cfg: self.write_register(servo_id, 23, cfg['d_coefficient'], 1)
                    
                    # Write Acceleration limits if applicable
                    if 'acceleration' in cfg: self.write_register(servo_id, 41, cfg['acceleration'], 1)
                    if 'return_delay_time' in cfg: self.write_register(servo_id, 5, cfg['return_delay_time'], 1)
                    
                    # Relock EEPROM protection (Register 55 = 1)
                    self.write_register(servo_id, 55, 1, 1)
                    
                except Exception as e:
                    self.get_logger().error(f"Could not write calibration sequence properties to Servo {servo_id}: {e}")
            self.get_logger().info("Hardware target configuration initialization cycle complete.")

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
        if not msg.points or not hasattr(self, 'ser') or not self.ser.is_open:
            return
        target_point = msg.points[-1]
        
        with self.serial_lock:
            for index, j_name in enumerate(msg.joint_names):
                clean_name = j_name.replace('_joint', '')
                if clean_name in self.joint_map:
                    servo_id = self.joint_map[clean_name]
                    pos_ticks = self.radians_to_ticks(target_point.positions[index], servo_id)
                    
                    val_h = (pos_ticks >> 8) & 0xFF
                    val_l = pos_ticks & 0xFF
                    
                    packet = bytearray([0xFF, 0xFF, servo_id, 0x05, 0x03, 42, val_h, val_l])
                    packet.append(self.calculate_checksum(packet))
                    try:
                        self.ser.write(packet)
                        time.sleep(0.001)
                    except Exception as e:
                        self.get_logger().error(f"Write error on target servo command thread {servo_id}: {e}")

    def query_telemetry_callback(self):
        if not self.joint_names or not hasattr(self, 'ser') or not self.ser.is_open:
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
                        raw_ticks = (resp[5] << 8) | resp[6]
                        raw_load = (resp[9] << 8) | resp[10]
                        volt = resp[11]
                        temp = resp[12]
                        raw_current = (resp[18] << 8) | resp[19]
                        
                        if raw_ticks > 32767:
                            ticks = raw_ticks - 65536
                        else:
                            ticks = raw_ticks
                        
                        if raw_current > 32767: 
                            raw_current -= 65536
                        current_ma = max(min(int(raw_current * 6.5), 32767), -32768)
                        
                        if raw_load >= 1024:
                            load_percent = -(raw_load - 1024) / 10.0
                        else:
                            load_percent = raw_load / 10.0
                        load_percent = max(min(int(load_percent), 32767), -32768)
                        
                        positions_rad.append(self.waveshare_ticks_to_radians(ticks, servo_id))
                        temps.append(temp)
                        volts.append(volt)
                        currents.append(current_ma)
                        loads.append(load_percent)
                        success_count += 1
                    else:
                        success_count -= 1
                except Exception:
                    success_count -= 1

        if success_count == len(self.active_ids):
            joint_msg.position = positions_rad
            joint_msg.velocity = [0.0] * len(self.joint_names)
            joint_msg.effort = [float(l) for l in loads]
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
