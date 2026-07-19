import serial
import time

def try_ping(baud):
    print(f"\n--- Testing at {baud} Baud ---")
    try:
        ser = serial.Serial('/dev/ttyWaveshare', baud, timeout=0.5)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.1)
        
        # Standard Feetech PING packet for Servo ID 1
        # [0xFF, 0xFF, ID, LENGTH, COMMAND] + CHECKSUM
        packet = bytearray([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])
        
        print("Clearing buffers...")
        ser.reset_input_buffer()
        
        print("Sending PING packet to Servo 1...")
        ser.write(packet)
        ser.flush()
        
        # Give the servo plenty of time to respond
        time.sleep(0.1)
        
        response = ser.read(ser.in_waiting or 10)
        if response:
            print(f"SUCCESS! Received {len(response)} bytes back: {response.hex().upper()}")
        else:
            print("No response received (0 bytes).")
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

try_ping(115200)
try_ping(1000000)