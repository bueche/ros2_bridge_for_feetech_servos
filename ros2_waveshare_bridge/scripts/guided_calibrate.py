#!/usr/bin/env python3
import argparse
import serial
import time
import sys

parser = argparse.ArgumentParser(description="SO-ARM101 guided calibrator")
parser.add_argument('--ranges-only', action='store_true',
                     help="Skip homing-offset (re)calibration entirely. Use this once "
                          "the homing offsets are already confirmed correct -- reads "
                          "whatever offset is currently flashed on the hardware for "
                          "reporting, and goes straight to the range_min/range_max sweep.")
args = parser.parse_args()

PORT = '/dev/ttyWaveshare'
# BAUD = 115200  
BAUD = 1000000  


# Mapping of joint names to their physical servo IDs
JOINTS = [
    {"name": "shoulder_pan",  "id": 1},
    {"name": "shoulder_lift", "id": 2},
    {"name": "elbow_flex",    "id": 3},
    {"name": "wrist_flex",    "id": 4},
    {"name": "wrist_roll",    "id": 5},
    {"name": "gripper",       "id": 6}
]

def open_serial():
    try:
        return serial.Serial(PORT, BAUD, timeout=0.03)
    except Exception as e:
        print(f"Error opening serial port {PORT}: {e}")
        print("Ensure the main ROS 2 bridge node is stopped before running this helper!")
        sys.exit(1)

ser = open_serial()

def calculate_checksum(packet_bytes):
    return (~sum(packet_bytes[2:])) & 0xFF

def encode_sign_magnitude(value, sign_bit_index):
    # Feetech SMS/STS signed registers (Homing_Offset, Present_Position, etc.) are
    # sign-magnitude, NOT two's complement: bit `sign_bit_index` is a pure sign
    # flag, and the bits below it hold the unsigned magnitude. Must stay in sync
    # with the identical helper in ros2_waveshare_bridge.py.
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = abs(value)
    if magnitude > max_magnitude:
        raise ValueError(f"magnitude {magnitude} exceeds {max_magnitude} for sign_bit_index={sign_bit_index}")
    return magnitude | (1 << sign_bit_index) if value < 0 else magnitude

def decode_sign_magnitude(encoded, sign_bit_index):
    sign_mask = 1 << sign_bit_index
    magnitude = encoded & (sign_mask - 1)
    return -magnitude if (encoded & sign_mask) else magnitude

def read_register(servo_id, reg_address, num_bytes=1, sign_bit_index=None):
    packet = bytearray([0xFF, 0xFF, servo_id, 0x04, 0x02, reg_address, num_bytes])
    packet.append(calculate_checksum(packet))

    ser.reset_input_buffer()
    ser.write(packet)

    expected_len = 6 + num_bytes
    response = ser.read(expected_len)
    if len(response) == expected_len and response[0] == 0xFF and response[1] == 0xFF:
        if num_bytes == 1:
            raw_val = response[5]
        else:
            raw_val = response[5] | (response[6] << 8)  # Little-Endian
        if sign_bit_index is not None:
            return decode_sign_magnitude(raw_val, sign_bit_index)
        return raw_val
    return None

def read_raw_position(servo_id):
    # Register 56 = Present Position (2 bytes). Sign-magnitude, bit 15 = sign --
    # matters once a joint's corrected position crosses zero (e.g. wrist_roll).
    return read_register(servo_id, 56, 2, sign_bit_index=15)

def write_register(servo_id, reg_address, value, num_bytes=1, is_eeprom=True, sign_bit_index=None):
    if is_eeprom:
        _write_raw(servo_id, 55, 0, 1)  # unlock EEPROM
        time.sleep(0.01)
    _write_raw(servo_id, reg_address, value, num_bytes, sign_bit_index)
    time.sleep(0.01)
    if is_eeprom:
        _write_raw(servo_id, 55, 1, 1)  # lock EEPROM
        time.sleep(0.01)

def _write_raw(servo_id, reg_address, value, num_bytes, sign_bit_index=None):
    packet = bytearray([0xFF, 0xFF, servo_id])
    length = 3 + num_bytes  # matches ros2_waveshare_bridge.py's write_register exactly
    packet.append(length)
    packet.append(0x03)  # WRITE instruction
    packet.append(reg_address)
    if num_bytes == 1:
        packet.append(value & 0xFF)
    else:
        val_16 = encode_sign_magnitude(value, sign_bit_index) if sign_bit_index is not None else (value & 0xFFFF)
        packet.append(val_16 & 0xFF)
        packet.append((val_16 >> 8) & 0xFF)
    packet.append(calculate_checksum(packet))
    ser.write(packet)
    time.sleep(0.005)

def disable_torque(servo_id):
    _write_raw(servo_id, 40, 0, 1)

def reset_homing_offset(servo_id):
    # Zero out whatever offset is currently flashed, so Present_Position reflects
    # the raw physical encoder value during Step 1. (0 is identical in
    # sign-magnitude and two's complement, so this particular write needs no
    # special-casing.)
    write_register(servo_id, 31, 0, 2, is_eeprom=True, sign_bit_index=11)

def flash_homing_offset(servo_id, offset):
    write_register(servo_id, 31, offset, 2, is_eeprom=True, sign_bit_index=11)

# 1. Start by relaxing all joints
print("=" * 60)
print("             SO-ARM101 INTERACTIVE GUIDED CALIBRATOR")
print("=" * 60)
print("Relaxing all joints...")
for joint in JOINTS:
    disable_torque(joint["id"])

if args.ranges_only:
    print("\n--ranges-only: skipping homing-offset calibration.")
    print("Reading currently-flashed homing offsets for reporting...")
    calculated_offsets = {}
    for joint in JOINTS:
        current = read_register(joint["id"], 31, 2, sign_bit_index=11)
        if current is None:
            print(f"ERROR: Could not read homing offset for {joint['name']} (ID {joint['id']})!")
            sys.exit(1)
        calculated_offsets[joint["name"]] = current
        print(f"  {joint['name']:15s}: current homing_offset on hardware = {current}")

else:
    print("Clearing prior homing offsets...")
    for joint in JOINTS:
        reset_homing_offset(joint["id"])

    print("\nStep 1: Calibrate Homing Offsets")
    print("-" * 40)
    print("-> Manually pose your entire robotic arm into its perfect 'HOME' configuration.")
    print("   (Base centered, arm segments vertical/perpendicular, gripper closed).")
    input("\nWhen the arm is physically in position, press [ENTER] to save Home...")

    # Capture Home ticks (raw encoder position, since offsets were just zeroed above)
    home_ticks = {}
    for joint in JOINTS:
        val = read_raw_position(joint["id"])
        if val is None:
            print(f"ERROR: Could not communicate with {joint['name']} (ID {joint['id']})!")
            print("Please check serial connection and try again.")
            sys.exit(1)
        home_ticks[joint["name"]] = val

    # Compute offsets. Present_Position = raw - homing_offset (confirmed against real
    # hardware), so to land Present_Position on 2048: homing_offset = raw - 2048.
    # (NOT 2048 - raw -- that's the inverted formula that caused the original
    # calibration bug.)
    calculated_offsets = {}
    for joint in JOINTS:
        raw = home_ticks[joint["name"]]
        offset = raw - 2048
        calculated_offsets[joint["name"]] = offset
        print(f"  {joint['name']:15s}: Measured {raw:4d} ticks -> Homing Offset: {offset:4d}")

    print("\nFlashing homing offsets to EEPROM...")
    for joint in JOINTS:
        flash_homing_offset(joint["id"], calculated_offsets[joint["name"]])
    print("Done.")

    print("\n" + "!" * 60)
    print("ACTION REQUIRED: unplug the 12V power supply from the robot for")
    print("15 seconds, then plug it back in. This lets the new homing offsets")
    print("actually take effect before we measure range limits against them.")
    print("!" * 60)
    input("\nPress [ENTER] once power has been cycled and the robot is back on...")

    # Reconnect -- the serial port itself doesn't need the servos powered, but give
    # the servos a moment to boot before talking to them again.
    ser.close()
    time.sleep(1.0)
    ser = open_serial()
    time.sleep(0.2)

    print("Re-relaxing all joints for range calibration...")
    for joint in JOINTS:
        disable_torque(joint["id"])

print("\nStep 2: Calibrate Range Limits")
print("-" * 40)
print("We will now guide you through each joint individually to record safety limits.")
print("(These are now measured in the corrected coordinate space, post-offset.)")

range_limits = {}
for joint in JOINTS:
    name = joint["name"]
    print(f"\n>>> Target Joint: {name.upper()} (ID {joint['id']})")

    # Capture two physical extremes -- direction (which one is "negative"/"positive")
    # isn't established yet at this point in calibration, so don't assume the order
    # you move the joint in tells you which is min vs max. Sort numerically instead.
    print(f"  1. Move '{name}' to one of its two safe physical extremes.")
    input("     Press [ENTER] when ready to capture this extreme...")
    extreme_a = read_raw_position(joint["id"])
    while extreme_a is None:
        print("     Error reading sensor. Trying again...")
        time.sleep(0.5)
        extreme_a = read_raw_position(joint["id"])
    print(f"     Captured: {extreme_a} ticks")

    print(f"  2. Move '{name}' to its other safe physical extreme.")
    input("     Press [ENTER] when ready to capture this extreme...")
    extreme_b = read_raw_position(joint["id"])
    while extreme_b is None:
        print("     Error reading sensor. Trying again...")
        time.sleep(0.5)
        extreme_b = read_raw_position(joint["id"])
    print(f"     Captured: {extreme_b} ticks")

    min_val, max_val = sorted((extreme_a, extreme_b))
    print(f"     -> range_min: {min_val}, range_max: {max_val}")

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
    print(f"    drive_mode: 0  # TODO: verify direction on the real joint before trusting this")
    print(f"    range_min: {r_min}")
    print(f"    range_max: {r_max}")
    print(f"    p_coefficient: 16")
    print(f"    i_coefficient: 0")
    print(f"    d_coefficient: 32")
    print(f"    return_delay_time: 0")
    print(f"    acceleration: 254\n")
print("-" * 60)
