# Servo Tools for Feetech STS3215

Small standalone scripts for bench-testing the STS3215 servos of the SO-ARM100 / SO-101 before assembly. They talk to the servo bus adapter directly with the Feetech SDK, so LeRobot is not required.

## Requirements

```bash
pip install feetech-servo-sdk pyserial
```

The servo power supply must be connected to the bus adapter; USB alone only powers the adapter chip. All scripts use 1 Mbps, the STS3215 factory default.

The port is the first argument: `COM10`-style on Windows, `/dev/ttyACM0`-style on Linux. On Windows, list ports with `python -m serial.tools.list_ports -v` and look for the CH340/CH343 adapter.

## Scripts

| Script | What it does | Writes to servo |
|---|---|---|
| `servo_monitor.py PORT [ID] [SECONDS]` | Live position, speed, load, voltage and temperature for one servo. If the servo stops answering, it rescans all IDs. | No |
| `servo_set_ids.py PORT [--dry-run]` | Assigns IDs one motor at a time, in LeRobot's order and numbering: gripper 6, wrist_roll 5, wrist_flex 4, elbow_flex 3, shoulder_lift 2, shoulder_pan 1. Connect only the prompted motor, then press Enter (`s` skips, `q` quits). `--dry-run` reports without writing. | Yes, ID in EEPROM |
| `servo_sweep.py PORT [IDS] [SECONDS]` | Sweeps one or more servos together between 90° and 270° (e.g. `1,2`). Torque is switched off on exit. | RAM only |

Stop any running script with `Ctrl+C`. Only one program can use the port at a time.

> [!WARNING]
> `servo_sweep.py` turns each motor through 180°. Only run it on loose motors. In an assembled arm it would drive the joints into their hard stops.

### SO-101 leader arm gear ratios

When assigning IDs on the leader arm, match the motor to the joint:

| Joint | Motor |
|---|---|
| shoulder_pan, elbow_flex | 1/191 (C044) |
| shoulder_lift | 1/345 (C001) |
| wrist_flex, wrist_roll, gripper | 1/147 (C046) |
