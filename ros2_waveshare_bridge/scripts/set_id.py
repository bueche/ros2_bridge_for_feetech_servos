#!/usr/bin/env python3
import argparse
import serial
import time

# Feetech / Waveshare STS Bus Constants
REG_LOCK = 55       # EEPROM Lock register (1 = Locked, 0 = Unlocked)
REG_ID = 5          # Servo ID register in EEPROM
CMD_PING = 0x01     # Ping Command
CMD_WRITE = 0x03    # Write Command

def calculate_checksum(servo_id, length, command, parameters):
    """Calculate Feetech protocol 1.0 checksum."""
    total = servo_id + length + command + sum(parameters)
    return (~total) & 0xFF

def send_packet(ser, servo_id, command, parameters=None):
    """Constructs and transmits a Feetech serial packet, returning raw response bytes."""
    if parameters is None:
        parameters = []
    
    length = len(parameters) + 2  # Length = params count + CMD byte + Checksum byte
    checksum = calculate_checksum(servo_id, length, command, parameters)
    
    packet = bytearray([0xFF, 0xFF, servo_id, length, command] + parameters + [checksum])
    
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)  # Allow time for transmission & response
    
    return ser.read(ser.in_waiting or 10)

def ping_servo(ser, servo_id):
    """Pings a target servo ID and returns True if a valid response is received."""
    response = send_packet(ser, servo_id, CMD_PING)
    if response and len(response) >= 6 and response[0] == 0xFF and response[1] == 0xFF:
        return True
    return False

def write_register_1byte(ser, servo_id, register_address, value):
    """Writes a 1-byte value to a specified register on the target servo."""
    params = [register_address, value]
    send_packet(ser, servo_id, CMD_WRITE, params)
    time.sleep(0.05)

def change_servo_id(ser, old_id, new_id):
    """Unlocks EEPROM, writes new ID, and locks EEPROM again."""
    print(f"   -> Unlocking EEPROM on Servo {old_id}...")
    write_register_1byte(ser, old_id, REG_LOCK, 0)
    
    print(f"   -> Writing new ID {new_id} to Register {REG_ID}...")
    write_register_1byte(ser, old_id, REG_ID, new_id)
    
    print(f"   -> Locking EEPROM on Servo {new_id}...")
    write_register_1byte(ser, new_id, REG_LOCK, 1)

def main():
    parser = argparse.ArgumentParser(description="Change Feetech/Waveshare STS Servo ID")
    parser.add_argument("--port", default="/dev/ttyWaveshare", help="Serial port device")
    parser.add_argument("--baud", type=int, default=1000000, help="Baud rate (default: 1000000)")
    parser.add_argument("old_id", type=int, help="Initial Servo ID")
    parser.add_argument("new_id", type=int, help="Desired Servo ID")
    args = parser.parse_args()

    if args.old_id == args.new_id:
        print("Error: Initial and desired Servo IDs must be different.")
        return

    print(f"Connecting to {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.1)
    except Exception as e:
        print(f"Failed to open port {args.port}: {e}")
        return

    try:
        # Step 1: Ping initial ID to confirm presence
        print(f"\n[Step 1/4] Pinging initial Servo ID {args.old_id}...")
        if ping_servo(ser, args.old_id):
            print(f"   [CONFIRMED] Servo ID {args.old_id} is present.")
        else:
            print(f"   [FAILED] Servo ID {args.old_id} did not respond. Aborting.")
            return

        # Step 2: Ping target ID to confirm collision avoidance
        print(f"\n[Step 2/4] Pinging target Servo ID {args.new_id}...")
        if not ping_servo(ser, args.new_id):
            print(f"   [CONFIRMED] Servo ID {args.new_id} is clear (not present on bus).")
        else:
            print(f"   [FAILED] Servo ID {args.new_id} is ALREADY in use on this bus. Aborting.")
            return

        # Step 3: Change the ID via EEPROM registers
        print(f"\n[Step 3/4] Changing Servo ID from {args.old_id} to {args.new_id}...")
        change_servo_id(ser, args.old_id, args.new_id)

        # Step 4: Ping the new ID to verify success
        print(f"\n[Step 4/4] Pinging new Servo ID {args.new_id} to confirm change...")
        if ping_servo(ser, args.new_id):
            print(f"   [SUCCESS] Servo successfully changed to ID {args.new_id} and verified!")
        else:
            print(f"   [FAILED] Servo ID {args.new_id} did not respond after flash operation.")

    finally:
        ser.close()

if __name__ == "__main__":
    main()