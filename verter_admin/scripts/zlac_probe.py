#!/usr/bin/env python3
"""ZLAC8015D bench probe — pre-ROS hardware validation.

Standalone tool used during chassis bring-up to validate that:
  • the isolated USB↔RS-485 converter enumerates and works,
  • Modbus RTU comms with ZLAC8015D succeed at the configured baud / slave,
  • basic read / write operations on the driver are correct,
  • wheels spin in the expected direction at the expected RPM.

Run this BEFORE writing or running any ROS adapter for the chassis. If this
script can't talk to the driver, neither can a ROS node.

Requirements:
  pip install pymodbus>=3.0 pyserial

═══════════════════════════════════════════════════════════════════════════════
REGISTER MAP — TODO VERIFY against the ZLAC8015D user manual delivered with
the unit. The addresses below are typical for the ZLTECH ZLAC8015D family but
DO vary between firmware revisions. Adjust the `_REG_*` constants once the
manual is in hand. The script's structure does not depend on the exact values;
only the constants need to change.
═══════════════════════════════════════════════════════════════════════════════

Usage:
  # safe — read only, no motion
  python3 zlac_probe.py probe
  python3 zlac_probe.py read 0x202C
  python3 zlac_probe.py read-status

  # motion — wheels MUST be in the air or robot on blocks
  python3 zlac_probe.py enable
  python3 zlac_probe.py spin --motor both --rpm 20 --duration 2
  python3 zlac_probe.py spin --motor left --rpm -30 --duration 1
  python3 zlac_probe.py disable

  # recovery
  python3 zlac_probe.py clear-fault

Defaults: port=/dev/zlac_chassis baud=115200 slave=1. Override with --port,
--baud, --slave.

The script ALWAYS attempts a disable on exit (Ctrl-C, exception, normal
return). Do not run with the robot on the floor unless you can stop it
mechanically.
"""
from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Iterator

try:
    from pymodbus.client import ModbusSerialClient
    from pymodbus.exceptions import ModbusException
    _PYMODBUS_AVAILABLE = True
except ImportError:
    _PYMODBUS_AVAILABLE = False
    ModbusSerialClient = None  # type: ignore[assignment]

    class ModbusException(Exception):  # type: ignore[no-redef]
        """Placeholder so type references resolve when pymodbus is missing."""


def _require_pymodbus() -> None:
    if not _PYMODBUS_AVAILABLE:
        sys.stderr.write(
            "ERROR: pymodbus not installed.  pip install 'pymodbus>=3.0' pyserial\n"
        )
        sys.exit(2)


# ═══════════════════════════════════════════════════════════════════════════
# REGISTER MAP — verify against ZLAC8015D manual
# ═══════════════════════════════════════════════════════════════════════════

# Driver identification
_REG_DEVICE_ID = 0x0000        # product code / model (R)
_REG_FW_VERSION = 0x0001       # firmware version (R)

# Operating mode (write once at setup)
_REG_OPERATION_MODE = 0x200D   # 1=position, 2=??, 3=velocity (verify in manual)
_MODE_VELOCITY = 3             # value to write for closed-loop velocity control

# Control word — enable / disable / fault clear
_REG_CONTROL_WORD = 0x200E
_CTRL_ENABLE = 0x0008          # both channels enabled (verify in manual)
_CTRL_DISABLE = 0x0007         # both channels disabled
_CTRL_CLEAR_FAULT = 0x0006     # clear latched fault (verify in manual)
_CTRL_EMERGENCY_STOP = 0x0005  # immediate hard stop

# Target velocity (signed int16, RPM, one register per motor)
_REG_TARGET_RPM_M1 = 0x2088    # left motor target (W)
_REG_TARGET_RPM_M2 = 0x2089    # right motor target (W)

# Feedback — actual RPM (R, signed int16)
_REG_ACTUAL_RPM_M1 = 0x20AB
_REG_ACTUAL_RPM_M2 = 0x20AC

# Feedback — accumulated position (R, two registers high+low → int32)
_REG_POSITION_M1_HI = 0x20A7
_REG_POSITION_M1_LO = 0x20A8
_REG_POSITION_M2_HI = 0x20A9
_REG_POSITION_M2_LO = 0x20AA

# Driver status / fault
_REG_DRIVER_STATUS = 0x20A5    # status bits / running mode
_REG_FAULT_CODE = 0x20A6       # 0 = no fault

# Comm-loss watchdog timeout (ms, W). Driver brakes if no Modbus traffic for N ms.
_REG_COMM_WATCHDOG_MS = 0x2007


# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════


def _to_signed16(raw: int) -> int:
    """Modbus returns uint16; convert two's-complement to signed."""
    return raw - 0x10000 if raw & 0x8000 else raw


def _to_uint16(value: int) -> int:
    """Clamp signed → uint16 for Modbus write."""
    if value < -0x8000 or value > 0x7FFF:
        raise ValueError(f"value {value} outside int16 range")
    return value & 0xFFFF


def _combine_int32(hi: int, lo: int) -> int:
    """Combine two uint16 registers (high, low) into a signed int32."""
    raw = (hi << 16) | lo
    return raw - 0x100000000 if raw & 0x80000000 else raw


# ═══════════════════════════════════════════════════════════════════════════
# ZLAC client
# ═══════════════════════════════════════════════════════════════════════════


@dataclass
class DriverStatus:
    fw_version: int | None
    status: int | None
    fault_code: int | None
    actual_rpm_m1: int | None
    actual_rpm_m2: int | None
    position_m1: int | None
    position_m2: int | None


class ZLACClient:
    """Thin wrapper around pymodbus ModbusSerialClient with ZLAC semantics."""

    def __init__(self, port: str, baud: int, slave: int, timeout: float = 0.2) -> None:
        self.slave = slave
        self._client = ModbusSerialClient(
            port=port,
            baudrate=baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=timeout,
        )

    def connect(self) -> bool:
        return self._client.connect()

    def close(self) -> None:
        self._client.close()

    # ----- low-level

    def read_register(self, address: int) -> int:
        result = self._client.read_holding_registers(
            address=address, count=1, slave=self.slave
        )
        if result.isError():
            raise ModbusException(f"read 0x{address:04X} failed: {result}")
        return result.registers[0]

    def read_registers(self, address: int, count: int) -> list[int]:
        result = self._client.read_holding_registers(
            address=address, count=count, slave=self.slave
        )
        if result.isError():
            raise ModbusException(
                f"read {count}@0x{address:04X} failed: {result}"
            )
        return list(result.registers)

    def write_register(self, address: int, value: int) -> None:
        result = self._client.write_register(
            address=address, value=value, slave=self.slave
        )
        if result.isError():
            raise ModbusException(
                f"write 0x{address:04X}=0x{value:04X} failed: {result}"
            )

    # ----- high-level

    def read_status(self) -> DriverStatus:
        def safe(fn):
            try:
                return fn()
            except ModbusException as e:
                print(f"  (read failed: {e})")
                return None

        fw = safe(lambda: self.read_register(_REG_FW_VERSION))
        status = safe(lambda: self.read_register(_REG_DRIVER_STATUS))
        fault = safe(lambda: self.read_register(_REG_FAULT_CODE))
        m1_rpm = safe(lambda: _to_signed16(self.read_register(_REG_ACTUAL_RPM_M1)))
        m2_rpm = safe(lambda: _to_signed16(self.read_register(_REG_ACTUAL_RPM_M2)))

        def safe_pos(hi_reg: int, lo_reg: int) -> int | None:
            try:
                regs = self.read_registers(hi_reg, 2)
                return _combine_int32(regs[0], regs[1])
            except ModbusException as e:
                print(f"  (position read failed: {e})")
                return None

        pos1 = safe_pos(_REG_POSITION_M1_HI, _REG_POSITION_M1_LO)
        pos2 = safe_pos(_REG_POSITION_M2_HI, _REG_POSITION_M2_LO)

        return DriverStatus(
            fw_version=fw,
            status=status,
            fault_code=fault,
            actual_rpm_m1=m1_rpm,
            actual_rpm_m2=m2_rpm,
            position_m1=pos1,
            position_m2=pos2,
        )

    def set_velocity_mode(self) -> None:
        self.write_register(_REG_OPERATION_MODE, _MODE_VELOCITY)

    def enable(self) -> None:
        self.write_register(_REG_CONTROL_WORD, _CTRL_ENABLE)

    def disable(self) -> None:
        self.write_register(_REG_CONTROL_WORD, _CTRL_DISABLE)

    def clear_fault(self) -> None:
        self.write_register(_REG_CONTROL_WORD, _CTRL_CLEAR_FAULT)

    def set_target_rpm(self, m1: int, m2: int) -> None:
        self.write_register(_REG_TARGET_RPM_M1, _to_uint16(m1))
        self.write_register(_REG_TARGET_RPM_M2, _to_uint16(m2))

    def stop(self) -> None:
        try:
            self.set_target_rpm(0, 0)
        except ModbusException:
            pass


@contextmanager
def open_driver(port: str, baud: int, slave: int) -> Iterator[ZLACClient]:
    """Open ZLAC connection with guaranteed disable-on-exit safety net."""
    _require_pymodbus()
    client = ZLACClient(port=port, baud=baud, slave=slave)
    if not client.connect():
        raise SystemExit(f"ERROR: could not open serial port {port}")

    def panic_disable() -> None:
        try:
            client.stop()
            client.disable()
        except Exception:
            pass

    atexit.register(panic_disable)

    def on_signal(signum, _frame):
        panic_disable()
        sys.exit(130)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        yield client
    finally:
        panic_disable()
        client.close()


# ═══════════════════════════════════════════════════════════════════════════
# Subcommands
# ═══════════════════════════════════════════════════════════════════════════


def _print_status(s: DriverStatus) -> None:
    def fmt(v):
        return "—" if v is None else (f"0x{v:04X}" if isinstance(v, int) and v >= 0 else str(v))

    print(f"  fw_version:    {fmt(s.fw_version)}")
    print(f"  status:        {fmt(s.status)}")
    print(
        f"  fault_code:    {fmt(s.fault_code)}"
        + ("  ← FAULT" if s.fault_code not in (None, 0) else "")
    )
    print(f"  M1 actual RPM: {s.actual_rpm_m1}")
    print(f"  M2 actual RPM: {s.actual_rpm_m2}")
    print(f"  M1 position:   {s.position_m1}")
    print(f"  M2 position:   {s.position_m2}")


def cmd_probe(args: argparse.Namespace) -> int:
    print(f"Opening {args.port} @ {args.baud} 8N1 slave={args.slave} …")
    with open_driver(args.port, args.baud, args.slave) as drv:
        print("OK — connection established")
        print("Reading driver status:")
        _print_status(drv.read_status())
    return 0


def cmd_read(args: argparse.Namespace) -> int:
    addr = int(args.address, 0)  # supports 0x... and decimal
    with open_driver(args.port, args.baud, args.slave) as drv:
        value = drv.read_register(addr)
        print(f"  0x{addr:04X} = 0x{value:04X} ({value}, signed={_to_signed16(value)})")
    return 0


def cmd_read_status(args: argparse.Namespace) -> int:
    with open_driver(args.port, args.baud, args.slave) as drv:
        _print_status(drv.read_status())
    return 0


def cmd_enable(args: argparse.Namespace) -> int:
    with open_driver(args.port, args.baud, args.slave) as drv:
        drv.set_velocity_mode()
        drv.enable()
        print("enable: control word written. Re-reading status:")
        _print_status(drv.read_status())
        # IMPORTANT: open_driver's exit handler will immediately disable on
        # return.  The "enable then exit" pattern doesn't latch on, since on
        # process exit we shut the drive down. The state persists only while
        # this process is alive. Re-enable after the prompt to allow holding:
        if args.hold:
            print("\nHolding enabled — press Ctrl-C to disable and exit.")
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                print("\nDisabling.")
    return 0


def cmd_disable(args: argparse.Namespace) -> int:
    with open_driver(args.port, args.baud, args.slave) as drv:
        drv.disable()
        print("disable: control word written.")
    return 0


def cmd_clear_fault(args: argparse.Namespace) -> int:
    with open_driver(args.port, args.baud, args.slave) as drv:
        drv.clear_fault()
        time.sleep(0.05)
        s = drv.read_status()
        print(
            f"fault_code after clear: "
            f"{'0x%04X' % s.fault_code if s.fault_code is not None else '—'}"
        )
        if s.fault_code not in (None, 0):
            print("Fault persists — root cause not removed.")
            return 1
    return 0


def cmd_spin(args: argparse.Namespace) -> int:
    if args.duration <= 0 or args.duration > 30:
        sys.stderr.write("duration must be in (0, 30] seconds for bench safety\n")
        return 2
    if abs(args.rpm) > 250:
        sys.stderr.write(
            f"|rpm|={abs(args.rpm)} exceeds 250 — refusing (motor rated 200 RPM)\n"
        )
        return 2

    m1 = args.rpm if args.motor in ("left", "both") else 0
    m2 = args.rpm if args.motor in ("right", "both") else 0
    print(f"Spinning M1={m1}rpm M2={m2}rpm for {args.duration:.1f}s …")

    with open_driver(args.port, args.baud, args.slave) as drv:
        drv.set_velocity_mode()
        drv.enable()
        time.sleep(0.05)
        drv.set_target_rpm(m1, m2)

        t_end = time.monotonic() + args.duration
        next_print = 0.0
        while time.monotonic() < t_end:
            if time.monotonic() >= next_print:
                s = drv.read_status()
                print(
                    f"  t={time.monotonic():.2f}  "
                    f"M1: tgt={m1:4d} act={s.actual_rpm_m1}  "
                    f"M2: tgt={m2:4d} act={s.actual_rpm_m2}  "
                    f"fault={s.fault_code}"
                )
                next_print = time.monotonic() + 0.2
            time.sleep(0.02)

        drv.set_target_rpm(0, 0)
        time.sleep(0.1)
        print("Stop.  Final status:")
        _print_status(drv.read_status())
    return 0


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="zlac_probe",
        description="ZLAC8015D bench probe — pre-ROS hardware validation.",
    )
    p.add_argument("--port", default="/dev/zlac_chassis", help="serial port")
    p.add_argument("--baud", type=int, default=115200, help="Modbus baud rate")
    p.add_argument("--slave", type=int, default=1, help="Modbus slave address")

    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("probe", help="open port, read version + status + fault")
    sub.add_parser("read-status", help="read full status block")

    rd = sub.add_parser("read", help="read one holding register")
    rd.add_argument("address", help="register address (decimal or 0xNNNN)")

    en = sub.add_parser("enable", help="enable both channels (velocity mode)")
    en.add_argument(
        "--hold",
        action="store_true",
        help="keep process alive so enable stays on (Ctrl-C to disable)",
    )
    sub.add_parser("disable", help="disable both channels")
    sub.add_parser("clear-fault", help="clear latched fault and re-read status")

    sp = sub.add_parser("spin", help="drive motor(s) at target RPM for N seconds")
    sp.add_argument("--motor", choices=("left", "right", "both"), required=True)
    sp.add_argument("--rpm", type=int, required=True, help="signed target RPM")
    sp.add_argument("--duration", type=float, required=True, help="seconds (0, 30]")

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _require_pymodbus()
    dispatch = {
        "probe": cmd_probe,
        "read": cmd_read,
        "read-status": cmd_read_status,
        "enable": cmd_enable,
        "disable": cmd_disable,
        "clear-fault": cmd_clear_fault,
        "spin": cmd_spin,
    }
    try:
        return dispatch[args.cmd](args)
    except ModbusException as e:
        print(f"Modbus error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
