python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyWaveshare', 115200, timeout=0.03)

def disable_torque(servo_id):
    # Packet: [Header, Header, ID, Length, WRITE_CMD, Reg_40 (Torque), Value_0, Checksum]
    packet = bytearray([0xFF, 0xFF, servo_id, 0x05, 0x03, 40, 0])
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    ser.write(packet)
    time.sleep(0.005)

print('Releasing torque on all 6 joints...')
for i in range(1, 7):
    disable_torque(i)
print('Arm is now limp and safe to pose manually!')
ser.close()
"
