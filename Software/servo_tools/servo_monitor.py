"""Continuously monitor a Feetech STS3215 servo (read-only; never writes to the servo).

Usage: python servo_monitor.py [PORT] [ID] [SECONDS]
Stop with Ctrl+C.
"""
import shutil
import sys
import time

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
SERVO_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else None  # None = run forever
BAUD = 1_000_000

# STS3215 control table (RAM, read-only use)
ADDR_POSITION, ADDR_SPEED, ADDR_LOAD, ADDR_VOLTAGE, ADDR_TEMP, ADDR_MOVING = 56, 58, 60, 62, 63, 66

port = PortHandler(PORT)
pkt = PacketHandler(0)  # protocol 0 = STS/SMS series
if not port.openPort():
    sys.exit(f"Could not open {PORT} (is another program using it?)")
port.setBaudRate(BAUD)

live = sys.stdout.isatty()


def signed(value, sign_bit):
    return -(value & ~(1 << sign_bit)) if value & (1 << sign_bit) else value


def read_all(sid):
    pos, r, _ = pkt.read2ByteTxRx(port, sid, ADDR_POSITION)
    if r != COMM_SUCCESS:
        return None
    speed, _, _ = pkt.read2ByteTxRx(port, sid, ADDR_SPEED)
    load, _, _ = pkt.read2ByteTxRx(port, sid, ADDR_LOAD)
    volt, _, _ = pkt.read1ByteTxRx(port, sid, ADDR_VOLTAGE)
    temp, _, _ = pkt.read1ByteTxRx(port, sid, ADDR_TEMP)
    moving, _, err = pkt.read1ByteTxRx(port, sid, ADDR_MOVING)
    return pos, signed(speed, 15), signed(load, 10), volt / 10, temp, moving, err


def full_scan():
    return [i for i in range(254) if pkt.ping(port, i)[1] == COMM_SUCCESS]


print(f"Monitoring servo ID {SERVO_ID} on {PORT} @ {BAUD} bps  (Ctrl+C to stop)")
start = time.time()
polls = misses = 0
try:
    while DURATION is None or time.time() - start < DURATION:
        data = read_all(SERVO_ID)
        polls += 1
        stamp = time.strftime("%H:%M:%S")
        if data is None:
            misses += 1
            print(f"\n{stamp}  ID {SERVO_ID}: no response, scanning all IDs...", flush=True)
            ids = full_scan()
            print(f"{stamp}  responding IDs: {ids if ids else 'none'}", flush=True)
            if ids and SERVO_ID not in ids:
                SERVO_ID = ids[0]
                print(f"{stamp}  switching to ID {SERVO_ID}", flush=True)
            time.sleep(0.5)
            continue
        pos, speed, load, volt, temp, moving, err = data
        # Most important fields first, so a narrow panel still shows them.
        line = (f"ID{SERVO_ID} {pos:4d} {pos * 360 / 4096:5.1f}deg {volt:4.1f}V {temp:2d}C "
                f"load {load / 10:5.1f}% spd {speed:5d} {'MOVE ' if moving else 'still'} "
                f"err{err} miss {misses}/{polls} {stamp}")
        if live:
            # Truncate to the panel width so \r redraws one line instead of wrapping.
            width = shutil.get_terminal_size((80, 24)).columns - 1
            print("\r" + line[:width].ljust(width), end="", flush=True)
        else:
            print(line, flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    port.closePort()
    print(f"\nStopped. {polls} polls, {misses} missed.")
