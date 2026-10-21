"""Assign SO-101 motor IDs one motor at a time (same order and numbering as LeRobot's setup-motors).

For each joint: connect ONLY that motor to the bus, press Enter, and its ID is written to EEPROM.
Usage: python servo_set_ids.py [PORT] [--dry-run]
"""
import sys

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = args[0] if args else "COM10"
DRY_RUN = "--dry-run" in sys.argv
BAUD = 1_000_000

JOINTS = [("gripper", 6), ("wrist_roll", 5), ("wrist_flex", 4),
          ("elbow_flex", 3), ("shoulder_lift", 2), ("shoulder_pan", 1)]

ADDR_ID, ADDR_TORQUE, ADDR_LOCK = 5, 40, 55

port = PortHandler(PORT)
pkt = PacketHandler(0)  # protocol 0 = STS/SMS series
if not port.openPort():
    sys.exit(f"Could not open {PORT} (is another program using it?)")
port.setBaudRate(BAUD)


def scan():
    return [i for i in range(254) if pkt.ping(port, i)[1] == COMM_SUCCESS]


def write1(sid, addr, value, what):
    r, e = pkt.write1ByteTxRx(port, sid, addr, value)
    if r != COMM_SUCCESS:
        raise RuntimeError(f"{what}: {pkt.getTxRxResult(r)}")
    if e:
        raise RuntimeError(f"{what}: servo error {pkt.getRxPacketError(e)}")


def set_id(old, new):
    write1(old, ADDR_TORQUE, 0, "disable torque")
    write1(old, ADDR_LOCK, 0, "unlock EEPROM")
    # Some firmware replies to the ID write from the new ID (or not at all), so don't
    # require a status packet here; the ping below is the real check.
    pkt.write1ByteTxRx(port, old, ADDR_ID, new)
    if pkt.ping(port, new)[1] != COMM_SUCCESS:
        raise RuntimeError(f"write ID: motor did not answer as ID {new}")
    write1(new, ADDR_LOCK, 1, "lock EEPROM")


print(f"Motor ID setup on {PORT} @ {BAUD} bps{'  [DRY RUN - nothing will be written]' if DRY_RUN else ''}")
print("Connect ONE motor at a time. Enter = go, s = skip this joint, q = quit.\n")
done = []
try:
    for joint, new_id in JOINTS:
        while True:
            answer = input(f"Connect only the '{joint}' motor (-> ID {new_id}) and press Enter: ").strip().lower()
            if answer in ("q", "s"):
                break
            ids = scan()
            if len(ids) != 1:
                print(f"  Expected exactly 1 motor, found {len(ids)} ({ids if ids else 'none'}). "
                      "Check that only this motor is connected and powered, then try again.")
                continue
            old = ids[0]
            if old == new_id:
                print(f"  Already ID {new_id}. Nothing to change.")
            elif DRY_RUN:
                print(f"  Would change ID {old} -> {new_id}.")
            else:
                set_id(old, new_id)
                ok = pkt.ping(port, new_id)[1] == COMM_SUCCESS and pkt.ping(port, old)[1] != COMM_SUCCESS
                if not ok:
                    raise RuntimeError(f"verify failed: ID {new_id} did not answer after the write")
                print(f"  '{joint}' motor: ID {old} -> {new_id}, verified.")
            done.append((joint, new_id))
            break
        if answer == "q":
            break
except (KeyboardInterrupt, EOFError):
    print()
except RuntimeError as exc:
    print(f"\nStopped: {exc}")
finally:
    port.closePort()
    print("\nDone:", ", ".join(f"{j}={i}" for j, i in done) if done else "no motors set")
