#!/usr/bin/env python3
import serial
import time
import sys

PORT = '/dev/ttyWaveshare'
BAUD = 115200

# Mapping of joint names to their physical servo IDs
JOINTS = [
    {"name": "shoulder_pan",  "id": 1},
    {"name": "shoulder_lift", "id": 2},
    {"name": "elbow_flex",    "id": 3},
    {"name": "wrist_flex",    "id": 4},
    {"name": "wrist_roll",    "id": 5},
    {"name": "gripper",       "id": 6}
]

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.03)
except Exception as e:
    print(f"Error opening serial port {PORT}: {e}")
    print("Ensure the main ROS 2 bridge node is stopped before running this helper!")
    sys.exit(1)

def calculate_checksum(packet_bytes):
    return (~sum(packet_bytes[2:])) & 0xFF

def read_raw_position(servo_id):
    # Register 56 = Present Position (2 bytes)
    packet = bytearray([0xFF, 0xFF, servo_id, 0x04, 0x02, 56, 2])
    packet.append(calculate_checksum(packet))
    
    ser.reset_input_buffer()
    ser.write(packet)
    
    response = ser.read(8)
    if len(response) == 8 and response[0] == 0xFF and response[1] == 0xFF:
        return (response[5] << 8) | response[6]
    return None

def disable_torque(servo_id):
    # Register 40 = Torque Enable
    packet = bytearray([0xFF, 0xFF, servo_id, 0x05, 0x03, 40, 0])
    packet.append(calculate_checksum(packet))
    ser.write(packet)
    time.sleep(0.005)

# 1. Start by relaxing all joints
print("=" * 60)
print("             SO-ARM101 INTERACTIVE GUIDED CALIBRATOR")
print("=" * 60)
print("Relaxing all joints to allow manual posing...")
for joint in JOINTS:
    disable_torque(joint["id"])

print("\nStep 1: Calibrate Homing Offsets")
print("-" * 40)
print("-> Manually pose your entire robotic arm into its perfect 'HOME' configuration.")
print("   (Base centered, arm segments vertical/perpendicular, gripper closed).")
input("\nWhen the arm is physically in position, press [ENTER] to save Home...")

# Capture Home ticks
home_ticks = {}
for joint in JOINTS:
    val = read_raw_position(joint["id"])
    if val is None:
        print(f"ERROR: Could not communicate with {joint['name']} (ID {joint['id']})!")
        print("Please check serial connection and try again.")
        sys.exit(1)
    home_ticks[joint["name"]] = val
    
# Compute offsets (Target 2048 - Raw Measured)
calculated_offsets = {}
for joint in JOINTS:
    raw = home_ticks[joint["name"]]
    # homing_offset is target (2048) - raw
    calculated_offsets[joint["name"]] = 2048 - raw
    print(f"  {joint['name']:15s}: Measured {raw:4d} ticks -> Homing Offset: {2048 - raw:4d}")

print("\nStep 2: Calibrate Range Limits")
print("-" * 40)
print("We will now guide you through each joint individually to record safety limits.")

range_limits = {}
for joint in JOINTS:
    name = joint["name"]
    print(f"\n>>> Target Joint: {name.upper()} (ID {joint['id']})")
    
    # Measure MIN limit
    print(f"  1. Move '{name}' to its maximum safe NEGATIVE (MIN) limit.")
    input("     Press [ENTER] when ready to capture MIN limit...")
    min_val = read_raw_position(joint["id"])
    while min_val is None:
        print("     Error reading sensor. Trying again...")
        time.sleep(0.5)
        min_val = read_raw_position(joint["id"])
    print(f"     Captured MIN: {min_val} ticks")
    
    # Measure MAX limit
    print(f"  2. Move '{name}' to its maximum safe POSITIVE (MAX) limit.")
    input("     Press [ENTER] when ready to capture MAX limit...")
    max_val = read_raw_position(joint["id"])
    while max_val is None:
        print("     Error reading sensor. Trying again...")
        time.sleep(0.5)
        max_val = read_raw_position(joint["id"])
    print(f"     Captured MAX: {max_val} ticks")
    
    range_limits[name] = {"min": min_val, "max": max_val}

# Close connection
ser.close()

# 3. Generate and print the complete, formatted YAML configuration block
print("\n" + "=" * 60)
print("                    CALIBRATION COMPLETE!")
print("=" * 60)
print("Copy and paste the block below directly into your 'so-arm101.yaml' file:")
print("-" * 60)
print("joints:")
for joint in JOINTS:
    name = joint["name"]
    o_id = joint["id"]
    offset = calculated_offsets[name]
    r_min = range_limits[name]["min"]
    r_max = range_limits[name]["max"]
    
    # Print clean YAML matching your so-arm101.yaml structure
    print(f"  {name}:")
    print(f"    id: {o_id}")
    print(f"    homing_offset: {offset}")
    print(f"    range_min: {r_min}")
    print(f"    range_max: {r_max}")
    print(f"    p_coefficient: 16")
    print(f"    i_coefficient: 0")
    print(f"    d_coefficient: 32")
    print(f"    return_delay_time: 0")
    print(f"    acceleration: 254\n")
print("-" * 60)
