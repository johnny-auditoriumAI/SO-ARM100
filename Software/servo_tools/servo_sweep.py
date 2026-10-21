"""Sweep one or more Feetech STS3215s back and forth together in position mode until Ctrl+C.

Only writes RAM registers (torque, accel, speed, goal position); nothing stored in EEPROM is changed.
Torque is always switched off on exit.

WARNING: sweeps 90 <-> 270 deg. Only run it on loose motors, not ones assembled into an arm.

Usage: python servo_sweep.py [PORT] [IDS, e.g. 1,2] [SECONDS]
"""
import sys
import time

from scservo_sdk import COMM_SUCCESS, GroupSyncWrite, PacketHandler, PortHandler

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
IDS = [int(i) for i in sys.argv[2].split(",")] if len(sys.argv) > 2 else [1]
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else None  # None = run until Ctrl+C
BAUD = 1_000_000

TARGETS = (1024, 3072)  # 90 deg and 270 deg
SPEED = 1000            # steps/s (~88 deg/s)
ACCEL = 50              # x100 steps/s^2
TOLERANCE = 20          # steps
MOVE_TIMEOUT = 5.0      # s

# STS3215 control table
ADDR_MIN_LIMIT, ADDR_MAX_LIMIT, ADDR_MODE = 9, 11, 33
ADDR_TORQUE, ADDR_ACCEL, ADDR_GOAL_POS, ADDR_GOAL_TIME, ADDR_GOAL_SPEED = 40, 41, 42, 44, 46
ADDR_POSITION, ADDR_LOAD, ADDR_VOLTAGE, ADDR_TEMP = 56, 60, 62, 63

port = PortHandler(PORT)
pkt = PacketHandler(0)  # protocol 0 = STS/SMS series
if not port.openPort():
    sys.exit(f"Could not open {PORT} (is another program, e.g. the monitor, using it?)")
port.setBaudRate(BAUD)
goal_writer = GroupSyncWrite(port, pkt, ADDR_GOAL_POS, 2)


def check(result, error, what):
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{what}: {pkt.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{what}: servo error {pkt.getRxPacketError(error)}")


def read1(sid, addr, what):
    value, r, e = pkt.read1ByteTxRx(port, sid, addr)
    check(r, e, f"ID {sid} {what}")
    return value


def read2(sid, addr, what):
    value, r, e = pkt.read2ByteTxRx(port, sid, addr)
    check(r, e, f"ID {sid} {what}")
    return value


def write1(sid, addr, value, what):
    r, e = pkt.write1ByteTxRx(port, sid, addr, value)
    check(r, e, f"ID {sid} {what}")


def write2(sid, addr, value, what):
    r, e = pkt.write2ByteTxRx(port, sid, addr, value)
    check(r, e, f"ID {sid} {what}")


def send_goals(goals):
    # One sync-write packet so all servos start moving at the same instant.
    goal_writer.clearParam()
    for sid, goal in goals.items():
        goal_writer.addParam(sid, [goal & 0xFF, (goal >> 8) & 0xFF])
    r = goal_writer.txPacket()
    if r != COMM_SUCCESS:
        raise RuntimeError(f"sync write goals: {pkt.getTxRxResult(r)}")


def signed(value, sign_bit):
    return -(value & ~(1 << sign_bit)) if value & (1 << sign_bit) else value


torque_on = []
moves = 0
try:
    limits = {}
    for sid in IDS:
        mode = read1(sid, ADDR_MODE, "read mode")
        if mode != 0:
            sys.exit(f"Servo {sid} is in mode {mode}, not position mode (0). Not moving anything.")
        limits[sid] = (read2(sid, ADDR_MIN_LIMIT, "read min limit"), read2(sid, ADDR_MAX_LIMIT, "read max limit"))
        pos = read2(sid, ADDR_POSITION, "read position")
        print(f"Servo {sid}: position mode, limits {limits[sid][0]}-{limits[sid][1]}, now at {pos}")
        # Hold the current position before enabling torque so it doesn't jump to a stale goal.
        write2(sid, ADDR_GOAL_POS, pos, "set goal to current")
        write2(sid, ADDR_GOAL_TIME, 0, "set goal time")
        write2(sid, ADDR_GOAL_SPEED, SPEED, "set speed")
        write1(sid, ADDR_ACCEL, ACCEL, "set accel")
    for sid in IDS:
        write1(sid, ADDR_TORQUE, 1, "enable torque")
        torque_on.append(sid)
    print(f"Sweeping IDs {IDS} together: {TARGETS[0]} <-> {TARGETS[1]} at {SPEED} steps/s  (Ctrl+C to stop)")

    start = time.time()
    while DURATION is None or time.time() - start < DURATION:
        goals = {sid: min(max(TARGETS[moves % 2], lo), hi) for sid, (lo, hi) in limits.items()}
        send_goals(goals)
        t0 = time.time()
        while True:
            positions = {sid: read2(sid, ADDR_POSITION, "read position") for sid in IDS}
            arrived = all(abs(positions[s] - goals[s]) <= TOLERANCE for s in IDS)
            if arrived or time.time() - t0 > MOVE_TIMEOUT:
                break
            time.sleep(0.02)
        moves += 1
        parts = []
        for sid in IDS:
            load = signed(read2(sid, ADDR_LOAD, "read load"), 10) / 10
            temp = read1(sid, ADDR_TEMP, "read temp")
            flag = "" if abs(positions[sid] - goals[sid]) <= TOLERANCE else " TIMEOUT"
            parts.append(f"ID{sid} {positions[sid]:4d} ({positions[sid] * 360 / 4096:5.1f}deg) "
                         f"load {load:5.1f}% {temp}C{flag}")
        volt = read1(IDS[0], ADDR_VOLTAGE, "read voltage") / 10
        print(f"{time.strftime('%H:%M:%S')} move {moves:3d} -> {goals[IDS[0]]:4d} {time.time() - t0:4.2f}s  "
              + " | ".join(parts) + f"  {volt:4.1f}V", flush=True)
        time.sleep(0.3)
except KeyboardInterrupt:
    pass
except RuntimeError as exc:
    print(f"\nStopping: {exc}")
finally:
    for sid in torque_on:
        r, e = pkt.write1ByteTxRx(port, sid, ADDR_TORQUE, 0)
        print(f"ID {sid}: torque off." if r == COMM_SUCCESS else f"WARNING: could not switch torque off on ID {sid}!")
    port.closePort()
    print(f"Stopped after {moves} moves.")
