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
    def __init__(self, init_serial=True):
        super().__init__('ros2_waveshare_bridge')
        
        # Declare Parameters
        self.declare_parameter('port', '/dev/ttyWaveshare')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('joint_config_file', '')
        self.declare_parameter('enable_tick_logging', False) # Diagnostic Flag
        self.declare_parameter('disable_torque', False) # Calibration Flag: keep torque off at boot instead of the normal on-at-boot behavior
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        yaml_path = self.get_parameter('joint_config_file').get_parameter_value().string_value
        self.log_ticks = self.get_parameter('enable_tick_logging').get_parameter_value().bool_value
        self.disable_torque = self.get_parameter('disable_torque').get_parameter_value().bool_value
        
        self.get_logger().info(f"URDF path =: '{urdf_path}'.")
        
        self.joint_map = {}
        self.joint_names = []
        self.active_ids = []
        self.servo_configs = {}

        # 1. Autonomously Parse URDF Target Layout
        if urdf_path and os.path.exists(urdf_path):
            self.parse_urdf_file(urdf_path)
        else:
            self.get_logger().error(f"URDF path missing or invalid: '{urdf_path}'. Waiting for configuration.")
            if init_serial:
                return

        # 2. Layer YAML modifications over the top if available
        if yaml_path and yaml_path.strip() and os.path.exists(yaml_path):
            self.parse_yaml_file(yaml_path)
            self.get_logger().info(f"Successfully loaded calibration file: {yaml_path}")
        else:
            self.get_logger().warn(f"No calibration YAML path provided or file missing. Operating on raw parameters. {yaml_path}")

        self.serial_lock = threading.Lock()
        self.ser = None
        if init_serial:
            try:
                self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.03, rtscts=False, dsrdtr=False)
                time.sleep(0.1)
                self.get_logger().info(f"Dynamic Bridge Active. Operating {len(self.joint_names)} joints at {baud} baud.")
                self.apply_servo_calibrations()
                self.set_torque_state(enable=not self.disable_torque)
            except Exception as e:
                self.get_logger().error(f"Failed to open port: {e}")
                return

        # Publishers & Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.trajectory_callback, 10
        )
        self.feetech_state_pub = self.create_publisher(FeetechState, '/feetech_state', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.telemetry_timer = self.create_timer(0.25, self.query_telemetry_callback)

    def parse_urdf_file(self, path):
        try:
            tree = ET.parse(path)
            root = tree.getroot()
            for joint in root.findall('joint'):
                j_name = joint.get('name')
                j_type = joint.get('type')
                if j_type in ['revolute', 'continuous']:
                    servo_id = None
                    params = {}
                    for param in joint.findall('.//param'):
                        p_name = param.get('name')
                        if p_name:
                            params[p_name] = param.text.strip() if param.text else ""
                    r2c = joint.find('ros2_control')
                    if r2c is not None:
                        for param in r2c.findall('param'):
                            if param.get('name') == 'servo_id':
                                servo_id = int(param.text.strip())
                            if param.get('name'):
                                params[param.get('name')] = param.text.strip() if param.text else ""
                    if servo_id is None:
                        for elem in joint.iter('param'):
                            if elem.get('name') == 'servo_id':
                                servo_id = int(elem.text.strip())
                            elif elem.get('name') == 'id':
                                servo_id = int(elem.text.strip())
                    if servo_id is not None:
                        self.joint_map[j_name] = servo_id
                        self.joint_names.append(j_name)
                        self.active_ids.append(servo_id)
                        offset_val = int(params.get('offset', 2048))
                        self.servo_configs[servo_id] = {
                            'homing_offset': int(params.get('homing_offset', offset_val - 2048)),
                            'p_coefficient': int(params.get('p_cofficient', params.get('p_coefficient', 16))),
                            'i_coefficient': int(params.get('i_coefficient', 0)),
                            'd_coefficient': int(params.get('d_coefficient', 32)),
                            'drive_mode': int(params.get('drive_mode', 0))  # 0=normal, 1=inverted (matches LeRobot's per-motor calibration field -- not derivable from servo id)
                        }
                        self.get_logger().info(f"Autonomously Mapped URDF Joint '{j_name}' to Servo ID {servo_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to automatically parse URDF file: {e}")

    def parse_yaml_file(self, path):
        try:
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
            if 'joints' in config:
                for j_name, data in config['joints'].items():
                    clean_name = j_name.replace('_joint', '')
                    if clean_name in self.joint_map:
                        servo_id = self.joint_map[clean_name]
                        for key in ['homing_offset', 'p_coefficient', 'i_coefficient', 'd_coefficient', 
                                    'acceleration', 'return_delay_time', 'max_torque_limit', 'drive_mode',
                                    'range_min', 'range_max']:
                            if key in data:
                                self.servo_configs[servo_id][key] = int(data[key])
                        self.get_logger().info(f"Merged YAML adjustments over Joint '{clean_name}' (ID: {servo_id})")
        except Exception as e:
            self.get_logger().error(f"Failed to process YAML file configuration adjustments: {e}")

    def calculate_checksum(self, packet_bytes):
        return (~sum(packet_bytes[2:])) & 0xFF

    @staticmethod
    def encode_sign_magnitude(value, sign_bit_index):
        # Feetech SMS/STS signed registers (Homing_Offset, Present_Position, etc.)
        # are sign-magnitude, NOT two's complement: bit `sign_bit_index` is a pure
        # sign flag, and the bits below it hold the unsigned magnitude.
        # Confirmed against the Feetech-derived encoding table used by LeRobot's
        # feetech.py driver (Homing_Offset uses sign_bit_index=11, i.e. bit 0x0800).
        max_magnitude = (1 << sign_bit_index) - 1
        magnitude = abs(value)
        if magnitude > max_magnitude:
            raise ValueError(f"magnitude {magnitude} exceeds {max_magnitude} for sign_bit_index={sign_bit_index}")
        return magnitude | (1 << sign_bit_index) if value < 0 else magnitude

    @staticmethod
    def decode_sign_magnitude(encoded, sign_bit_index):
        sign_mask = 1 << sign_bit_index
        magnitude = encoded & (sign_mask - 1)
        return -magnitude if (encoded & sign_mask) else magnitude

    def write_register(self, servo_id, reg_address, value, num_bytes=1, is_eeprom=True, sign_bit_index=None):
        packet = bytearray([0xFF, 0xFF, servo_id])
        length = 3 + num_bytes
        packet.append(length)
        cmd = 0x03  # WRITE command
        packet.append(cmd)
        packet.append(reg_address)
        
        if num_bytes == 1:
            packet.append(value & 0xFF)
        else:
            if sign_bit_index is not None:
                val_16 = self.encode_sign_magnitude(value, sign_bit_index)
            else:
                val_16 = value & 0xFFFF
            packet.append(val_16 & 0xFF)        # Low byte
            packet.append((val_16 >> 8) & 0xFF) # High byte
            
        packet.append(self.calculate_checksum(packet))
        
        with self.serial_lock:
            self.ser.reset_input_buffer()
            self.ser.write(packet)
            self.ser.flush()
            time.sleep(0.005) # Allow serial line and EEPROM write cycle to stabilize
            if self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)

    def read_register(self, servo_id, reg_address, num_bytes=1, sign_bit_index=None):
        packet = bytearray([0xFF, 0xFF, servo_id, 0x04, 0x02, reg_address, num_bytes])
        packet.append(self.calculate_checksum(packet))
        
        expected_length_field = 2 + num_bytes
        expected_len = 6 + num_bytes
        
        try:
            with self.serial_lock:
                self.ser.reset_input_buffer()
                self.ser.write(packet)
                
                # Fetch response with a generous buffer window
                resp = self.read_exact_bytes(expected_len + 10)
                if not resp or len(resp) < expected_len:
                    return None

                # Scan buffer for 0xFF 0xFF header alignment
                for i in range(len(resp) - expected_len + 1):
                    if resp[i] == 0xFF and resp[i+1] == 0xFF:
                        actual_id = resp[i+2]
                        actual_len = resp[i+3]
                        actual_err = resp[i+4]
                        
                        if actual_id == servo_id and actual_len == expected_length_field and actual_err == 0:
                            if num_bytes == 1:
                                return resp[i+5]
                            else:
                                raw_val = resp[i+5] | (resp[i+6] << 8)
                                if sign_bit_index is not None:
                                    return self.decode_sign_magnitude(raw_val, sign_bit_index)
                                return raw_val
        except Exception as e:
            self.get_logger().error(f"Read register exception on ID {servo_id}: {e}")
        return None

    def apply_servo_calibrations(self):
        self.get_logger().info("Checking hardware calibration state against target configuration...")
        dirty_eeprom_detected = False
        for servo_id, cfg in self.servo_configs.items():
            try:
                def verify_and_flash(reg, target_val, num_bytes, name, target_mem_is_eeprom=True, sign_bit_index=None):
                    nonlocal dirty_eeprom_detected
                    current_val = None
                    for attempt in range(3):
                        current_val = self.read_register(servo_id, reg, num_bytes, sign_bit_index=sign_bit_index)
                        if current_val is not None:
                            break
                        time.sleep(0.02)  # give a busy servo (e.g. mid EEPROM commit) a moment to recover
                    if current_val is None:
                         self.get_logger().warn(f"Servo {servo_id} {name} no current value after 3 attempts! Skipping this cycle.")
                         return
                    if current_val != target_val:
                        self.get_logger().warn(f"Servo {servo_id} {name} MISMATCH! Hardware: {current_val}, Config Target: {target_val}. Flashing...")
                        if target_mem_is_eeprom:
                            self.write_register(servo_id, 55, 0, 1, is_eeprom=False)
                            time.sleep(0.02)
                        self.write_register(servo_id, reg, target_val, num_bytes, is_eeprom=target_mem_is_eeprom, sign_bit_index=sign_bit_index)
                        time.sleep(0.02)
                        if target_mem_is_eeprom:
                            self.write_register(servo_id, 55, 1, 1, is_eeprom=False)
                            dirty_eeprom_detected = True
                            time.sleep(0.03)  # EEPROM commit settle time -- the next command (another
                                              # register's read/write) can otherwise land while this
                                              # servo is still busy committing and get no response.
                    else:
                        self.get_logger().info(f"Servo {servo_id} {name} matches hardware ({current_val}).")

                # 1. Homing Offset Validation (Register 31, sign-magnitude: bit 11 = sign, bits 0-10 = magnitude)
                if 'homing_offset' in cfg: verify_and_flash(31, cfg['homing_offset'], 2, "Homing Offset", target_mem_is_eeprom=True, sign_bit_index=11)
                if 'range_min' in cfg: verify_and_flash(9, cfg['range_min'], 2, "Min Angle Limit", target_mem_is_eeprom=True)
                if 'range_max' in cfg: verify_and_flash(11, cfg['range_max'], 2, "Max Angle Limit", target_mem_is_eeprom=True)
                if 'p_coefficient' in cfg: verify_and_flash(21, cfg['p_coefficient'], 1, "P Gain", target_mem_is_eeprom=True)
                if 'i_coefficient' in cfg: verify_and_flash(22, cfg['i_coefficient'], 1, "I Gain", target_mem_is_eeprom=True)
                if 'd_coefficient' in cfg: verify_and_flash(23, cfg['d_coefficient'], 1, "D Gain", target_mem_is_eeprom=True)
                if 'return_delay_time' in cfg: verify_and_flash(7, cfg['return_delay_time'], 1, "Return Delay", target_mem_is_eeprom=True)
                if 'acceleration' in cfg:     verify_and_flash(41, cfg['acceleration'], 1, "Acceleration", target_mem_is_eeprom=False)
            except Exception as e:
                self.get_logger().error(f"Could not complete validation matrix on Servo {servo_id}: {e}")
        if dirty_eeprom_detected:
            self.get_logger().error("--> ACTION: UNPLUG THE 12V POWER SUPPLY FROM THE ROBOT FOR 15 SECONDS NOW <--")
        else:
            self.get_logger().info("All active servo parameters match your YAML. Robot coordinate loop synchronized.")

    def set_torque_state(self, enable):
        state = 1 if enable else 0
        for servo_id in self.active_ids:
            try:
                self.write_register(servo_id, 40, state, 1, is_eeprom=False)
            except Exception as e:
                self.get_logger().error(f"Failed to set torque state on servo {servo_id}: {e}")
        self.get_logger().info(f"Torque {'ENABLED' if enable else 'DISABLED'} on {len(self.active_ids)} servos.")

    def read_exact_bytes(self, num_bytes):
        buffer = bytearray()
        start_time = time.time()
        while len(buffer) < num_bytes and (time.time() - start_time) < 0.04:
            chunk = self.ser.read(num_bytes - len(buffer))
            if chunk:
                buffer.extend(chunk)
        return buffer if len(buffer) == num_bytes else buffer

    def radians_to_ticks(self, rad, servo_id):
        inverted = self.servo_configs.get(servo_id, {}).get('drive_mode', 0) == 1
        if inverted:
            ticks = int(2048 - (rad * (2048.0 / math.pi)))
        else:
            ticks = int(2048 + (rad * (2048.0 / math.pi)))
        return max(min(ticks, 4095), 0)

    def waveshare_ticks_to_radians(self, ticks, servo_id):
        inverted = self.servo_configs.get(servo_id, {}).get('drive_mode', 0) == 1
        if inverted:
            return (2048 - ticks) * (math.pi / 2048.0)
        return (ticks - 2048) * (math.pi / 2048.0)

    def decode_telemetry_packet(self, resp, servo_id):
        if not resp or len(resp) != 21 or resp[0] != 0xFF or resp[1] != 0xFF:
            return None

        # Clean Little-Endian unpacking
        # Present_Position is sign-magnitude (bit 15 = sign), used for multi-turn
        # values that go negative -- NOT two's complement.
        raw_ticks = self.decode_sign_magnitude(resp[5] | (resp[6] << 8), 15)
        raw_load = resp[9] | (resp[10] << 8)
        volt = resp[11]
        temp = resp[12]
        raw_current = resp[18] | (resp[19] << 8)

        if self.log_ticks:
            self.get_logger().info(f"[DIAG] Servo ID {servo_id} -> Raw Ticks: {raw_ticks}")

        single_turn_ticks = raw_ticks % 4096
        position_rad = self.waveshare_ticks_to_radians(single_turn_ticks, servo_id)

        temperature = float(temp)
        voltage = float(volt)

        if raw_current > 32767:
            raw_current -= 65536
        current_ma = max(min(int(raw_current * 6.5), 32767), -32768)

        if raw_load > 32767:
            raw_load -= 65536
        if raw_load >= 1024:
            load_val = -(raw_load - 1024)
        elif raw_load <= -1024:
            load_val = -(raw_load + 1024)
        else:
            load_val = raw_load
        load_val = max(min(int(load_val), 32767), -32768)

        return {
            'position': position_rad,
            'temperature': temperature,
            'voltage': voltage,
            'current': current_ma,
            'load': load_val
        }

    def _query_single_servo_raw_bytes(self, servo_id):
        packet = bytearray([0xFF, 0xFF, servo_id, 0x04, 0x02, 56, 15])
        packet.append(self.calculate_checksum(packet))
        try:
            self.ser.reset_input_buffer()
            self.ser.write(packet)
            return self.read_exact_bytes(21)
        except Exception:
            return None

    def query_telemetry_callback(self):
        if not self.joint_names or not self.ser or not self.ser.is_open:
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
                resp = self._query_single_servo_raw_bytes(servo_id)
                data = self.decode_telemetry_packet(resp, servo_id)
                if data is not None:
                    positions_rad.append(data['position'])
                    temps.append(int(data['temperature']))
                    volts.append(int(data['voltage']))
                    currents.append(data['current'])
                    loads.append(data['load'])
                    success_count += 1
                else:
                    feetech_msg.comm_state = FeetechState.COMM_STATE_ITEM_READ_FAIL
                    positions_rad.append(0.0)
                    temps.append(0); volts.append(0); currents.append(0); loads.append(0)

        if success_count == len(self.active_ids):
            joint_msg.position = positions_rad
            joint_msg.velocity = [0.0] * len(self.joint_names)
            joint_msg.effort = [float(load_eff) for load_eff in loads]
            self.joint_state_pub.publish(joint_msg)

        feetech_msg.present_temperature = temps
        feetech_msg.present_input_voltage = volts
        feetech_msg.present_current = currents
        feetech_msg.present_load = loads
        self.feetech_state_pub.publish(feetech_msg)

    def trajectory_callback(self, msg):
        if not msg.points:
            return
        target_point = msg.points[-1]
        with self.serial_lock:
            for index, joint_name in enumerate(msg.joint_names):
                if joint_name in self.joint_map:
                    servo_id = self.joint_map[joint_name]
                    pos_ticks = self.radians_to_ticks(target_point.positions[index], servo_id)
                    
                    # Outbound position targets mapped as Little-Endian
                    val_l = pos_ticks & 0xFF
                    val_h = (pos_ticks >> 8) & 0xFF
                    
                    packet = bytearray([0xFF, 0xFF, servo_id, 0x05, 0x03, 42, val_l, val_h])
                    packet.append(self.calculate_checksum(packet))
                    try:
                        self.ser.write(packet)
                    except Exception as e:
                        self.get_logger().error(f"Write error: {e}")

    def destroy_node(self):
        # Always torque off on the way out, regardless of the disable_torque flag --
        # that flag only controls the boot-time state.
        if getattr(self, 'ser', None) and self.ser.is_open and getattr(self, 'active_ids', None):
            try:
                self.set_torque_state(enable=False)
            except Exception as e:
                self.get_logger().error(f"Failed to disable torque on shutdown: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Ros2WaveshareBridge(init_serial=True)
    try:
        if hasattr(node, 'ser') and node.ser:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
