#!/usr/bin/env python3
"""Measure how STS servos behave around an EEPROM write.

For each servo: torque off, unlock EEPROM, then N times write I_Coefficient
(reg 23) alternating between 1 and 0, so every write is a real EEPROM change
(rewriting an unchanged value may be skipped by the servo firmware), measuring
  reply_ms -- write() until the write's 6-byte status reply arrives
  busy_ms  -- after that reply, how long until the servo answers a
              Present_Position read again
Ends with I_Coefficient = 0 (the intended value), then locks EEPROM.
Torque is left OFF -- support the arm before running.
Run with nothing else on the port.

    python3 eeprom_write_timing.py [/dev/ttyWaveshare] [writes_per_servo]
"""
import statistics
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyWaveshare'
N = int(sys.argv[2]) if len(sys.argv) > 2 else 20
IDS = [1, 2, 3, 4, 5, 6]
REG_TORQUE, REG_LOCK, REG_I, REG_POS = 40, 55, 23, 56
GIVE_UP_S = 0.2
DRIVER_TIMEOUT_MS = 5.0
DRIVER_SETTLE_MS = 30.0

ser = serial.Serial(PORT, 1000000, timeout=0)


def checksum(pkt):
    return (~sum(pkt[2:])) & 0xFF


def transact(pkt, reply_len):
    """Send pkt, wait for reply_len bytes. Returns (elapsed_ms, bytes) or (None, partial)."""
    pkt.append(checksum(pkt))
    ser.reset_input_buffer()
    t0 = time.perf_counter()
    ser.write(pkt)
    ser.flush()
    buf = bytearray()
    while len(buf) < reply_len and time.perf_counter() - t0 < GIVE_UP_S:
        buf += ser.read(reply_len - len(buf))
    if len(buf) < reply_len:
        return None, buf
    return (time.perf_counter() - t0) * 1000.0, buf


def write1(sid, reg, val):
    return transact(bytearray([0xFF, 0xFF, sid, 0x04, 0x03, reg, val & 0xFF]), 6)


def read1(sid, reg):
    ms, buf = transact(bytearray([0xFF, 0xFF, sid, 0x04, 0x02, reg, 1]), 7)
    return None if ms is None else buf[5]


def busy_after(sid):
    """ms until the servo answers a 2-byte read again (polling every 0.5 ms)."""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < GIVE_UP_S:
        ms, _ = transact(bytearray([0xFF, 0xFF, sid, 0x04, 0x02, REG_POS, 2]), 8)
        if ms is not None and ms < 2.0:
            return (time.perf_counter() - t0) * 1000.0
        time.sleep(0.0005)
    return None


print(f"{'id':>3} {'I0':>3} {'reply med':>9} {'reply max':>9} {'>5ms':>5} {'no reply':>8} "
      f"{'busy med':>8} {'busy max':>8} {'>30ms':>6}  (ms)")
for sid in IDS:
    p = read1(sid, REG_I)
    if p is None:
        print(f"{sid:>3}  could not read I_Coefficient; skipping")
        continue
    write1(sid, REG_TORQUE, 0)
    time.sleep(0.05)
    write1(sid, REG_LOCK, 0)
    time.sleep(0.05)

    replies, busies, no_reply = [], [], 0
    for k in range(N):
        ms, _ = write1(sid, REG_I, 1 if k % 2 == 0 else 0)
        if ms is None:
            no_reply += 1
        else:
            replies.append(ms)
        b = busy_after(sid)
        if b is not None:
            busies.append(b)
        time.sleep(0.1)

    write1(sid, REG_I, 0)
    time.sleep(0.1)
    write1(sid, REG_LOCK, 1)
    time.sleep(0.1)

    def fmt(xs, f):
        return f"{f(xs):9.2f}" if xs else f"{'-':>9}"
    late = sum(1 for x in replies if x > DRIVER_TIMEOUT_MS)
    long_busy = sum(1 for x in busies if x > DRIVER_SETTLE_MS)
    print(f"{sid:>3} {p:>3} {fmt(replies, statistics.median)} {fmt(replies, max)} {late:>5} {no_reply:>8} "
          f"{fmt(busies, statistics.median)[1:]} {fmt(busies, max)[1:]} {long_busy:>6}")

ser.close()
print("I_Coefficient (reg 23) left at 0 and EEPROM re-locked on all servos; torque left OFF.\n"
      "I0 column = reg 23 value before the test.")
