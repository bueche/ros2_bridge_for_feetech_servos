#!/usr/bin/env python3
"""Measure servo reply latency on the Waveshare bus.

Sends a READ of Present_Position (reg 56, 2 bytes) to each servo N times and
records the time from write() until the full 8-byte status packet has arrived.
Run with nothing else on the port (no bridge, no ros2_control_node).

    python3 bus_latency.py [/dev/ttyWaveshare] [samples]
"""
import statistics
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyWaveshare'
SAMPLES = int(sys.argv[2]) if len(sys.argv) > 2 else 200
IDS = [1, 2, 3, 4, 5, 6]
DRIVER_TIMEOUT_MS = 5.0  # feetech_driver serial_port.hpp timeout_
REPLY_LEN = 8            # FF FF id len err lo hi chk
GIVE_UP_S = 0.1


def read_packet(servo_id):
    pkt = bytearray([0xFF, 0xFF, servo_id, 0x04, 0x02, 56, 2])
    pkt.append((~sum(pkt[2:])) & 0xFF)
    return pkt


ser = serial.Serial(PORT, 1000000, timeout=0)
results = {i: [] for i in IDS}
missing = {i: 0 for i in IDS}

for _ in range(SAMPLES):
    for sid in IDS:
        ser.reset_input_buffer()
        t0 = time.perf_counter()
        ser.write(read_packet(sid))
        ser.flush()
        buf = bytearray()
        while len(buf) < REPLY_LEN and time.perf_counter() - t0 < GIVE_UP_S:
            buf += ser.read(REPLY_LEN - len(buf))
        if len(buf) < REPLY_LEN:
            missing[sid] += 1
        else:
            results[sid].append((time.perf_counter() - t0) * 1000.0)
        time.sleep(0.002)

ser.close()

print(f"{'id':>3} {'n':>5} {'miss':>5} {'min':>7} {'median':>7} {'p99':>7} {'max':>7} {'>5ms':>6}  (ms)")
for sid in IDS:
    r = sorted(results[sid])
    if not r:
        print(f"{sid:>3} {0:>5} {missing[sid]:>5}  no replies")
        continue
    p99 = r[min(len(r) - 1, int(0.99 * len(r)))]
    late = sum(1 for x in r if x > DRIVER_TIMEOUT_MS)
    print(f"{sid:>3} {len(r):>5} {missing[sid]:>5} {r[0]:7.2f} {statistics.median(r):7.2f} "
          f"{p99:7.2f} {r[-1]:7.2f} {late:>6}")
