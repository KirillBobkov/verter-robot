"""ZLAC8015D bench probe — pre-ROS hardware validation CLI.

Lives inside the package (re-uses ZLACDriver) so there is one source of truth
for the Modbus protocol and so the CLI is installable via colcon:

    colcon build --packages-select verter_admin
    source install/setup.bash
    ros2 run verter_admin zlac_probe probe

Subcommands:
  probe                   open port, read fw version + status snapshot
  read 0x202C             read a single holding register (decimal or 0xNNNN)
  read-status             full feedback snapshot (RPM, position, fault)
  enable [--hold]         enable both channels (--hold keeps process alive)
  disable                 disable both channels
  clear-fault             clear latched driver fault
  spin --motor … --rpm … --duration …
                          drive motor(s) at target RPM for N seconds (wheels MUST
                          be in the air or robot on blocks)

Safety:
  * always attempts stop_motors() + disable() on exit (Ctrl-C, exception),
  * spin clamps |rpm| ≤ 250 and duration ≤ 30 s,
  * `enable` without --hold drops the drive at exit (the bench-safe default).
"""
from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from contextlib import contextmanager
from typing import Iterator

from .zlac_driver import WheelFeedback, ZLACDriver, ZLACDriverError


@contextmanager
def open_driver(port: str, baud: int, slave: int) -> Iterator[ZLACDriver]:
    """Open ZLAC connection with guaranteed stop+disable on exit."""
    driver = ZLACDriver(port=port, baud=baud, slave=slave)
    driver.open()

    def panic_disable() -> None:
        try:
            driver.stop_motors()
            driver.disable()
        except ZLACDriverError:
            pass

    atexit.register(panic_disable)

    def on_signal(signum, _frame):  # noqa: ARG001
        panic_disable()
        sys.exit(130)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        yield driver
    finally:
        panic_disable()
        driver.close()


# ─────────────────────────────────────────────────────────── printers ────


def _print_feedback(fb: WheelFeedback) -> None:
    fault_tag = "  ← FAULT" if fb.has_fault else ""
    print(f"  fault_code:    0x{fb.fault_code:04X}{fault_tag}")
    print(f"  M1 actual RPM: {fb.actual_rpm_left}")
    print(f"  M2 actual RPM: {fb.actual_rpm_right}")
    print(f"  M1 position:   {fb.position_left}")
    print(f"  M2 position:   {fb.position_right}")


def _safe_feedback(driver: ZLACDriver) -> None:
    """Read feedback and print, swallowing failures so probe can finish."""
    try:
        _print_feedback(driver.read_feedback())
    except ZLACDriverError as e:
        print(f"  feedback read failed: {e}")


# ────────────────────────────────────────────────────── subcommands ────


def cmd_probe(args: argparse.Namespace) -> int:
    print(f"Opening {args.port} @ {args.baud} 8N1 slave={args.slave} …")
    with open_driver(args.port, args.baud, args.slave) as drv:
        print("OK — connection established")
        try:
            fw = drv.read_fw_version()
            print(f"  fw_version:    0x{fw:04X}")
        except ZLACDriverError as e:
            print(f"  fw_version read failed: {e}")
        _safe_feedback(drv)
    return 0


def cmd_read(args: argparse.Namespace) -> int:
    addr = int(args.address, 0)
    with open_driver(args.port, args.baud, args.slave) as drv:
        value = drv.read_register(addr)
        signed = value - 0x10000 if value & 0x8000 else value
        print(f"  0x{addr:04X} = 0x{value:04X} ({value}, signed={signed})")
    return 0


def cmd_read_status(args: argparse.Namespace) -> int:
    with open_driver(args.port, args.baud, args.slave) as drv:
        _safe_feedback(drv)
    return 0


def cmd_enable(args: argparse.Namespace) -> int:
    with open_driver(args.port, args.baud, args.slave) as drv:
        drv.setup_velocity_mode(comm_watchdog_ms=args.watchdog_ms)
        drv.enable()
        print("enable: control word written.  Status:")
        _safe_feedback(drv)
        if args.hold:
            print("\nHolding enabled — Ctrl-C to disable and exit.")
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
        try:
            fault = drv.read_fault_only()
            print(f"fault_code after clear: 0x{fault:04X}")
            if fault != 0:
                print("Fault persists — root cause not removed.")
                return 1
        except ZLACDriverError as e:
            print(f"fault read failed: {e}")
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
        drv.setup_velocity_mode(comm_watchdog_ms=args.watchdog_ms)
        drv.enable()
        time.sleep(0.05)
        drv.write_target_rpm(m1, m2)

        t_end = time.monotonic() + args.duration
        next_print = 0.0
        while time.monotonic() < t_end:
            if time.monotonic() >= next_print:
                try:
                    fb = drv.read_feedback()
                    print(
                        f"  t={time.monotonic():.2f}  "
                        f"M1: tgt={m1:4d} act={fb.actual_rpm_left:4d}  "
                        f"M2: tgt={m2:4d} act={fb.actual_rpm_right:4d}  "
                        f"fault=0x{fb.fault_code:04X}"
                    )
                except ZLACDriverError as e:
                    print(f"  feedback failed: {e}")
                next_print = time.monotonic() + 0.2
            time.sleep(0.02)

        drv.write_target_rpm(0, 0)
        time.sleep(0.1)
        print("Stop.  Final status:")
        _safe_feedback(drv)
    return 0


# ────────────────────────────────────────────────────────── CLI ────────


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="zlac_probe",
        description="ZLAC8015D bench probe — pre-ROS hardware validation.",
    )
    p.add_argument("--port", default="/dev/zlac_chassis", help="serial port")
    p.add_argument("--baud", type=int, default=115200, help="Modbus baud rate")
    p.add_argument("--slave", type=int, default=1, help="Modbus slave address")
    p.add_argument(
        "--watchdog-ms",
        type=int,
        default=200,
        help="comm-loss watchdog (ms) applied on enable/spin",
    )

    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("probe", help="open port, read fw version + status")
    sub.add_parser("read-status", help="read full feedback snapshot")

    rd = sub.add_parser("read", help="read one holding register")
    rd.add_argument("address", help="register address (decimal or 0xNNNN)")

    en = sub.add_parser("enable", help="enable both channels (velocity mode)")
    en.add_argument(
        "--hold",
        action="store_true",
        help="keep process alive so enable persists (Ctrl-C to disable)",
    )
    sub.add_parser("disable", help="disable both channels")
    sub.add_parser("clear-fault", help="clear latched fault and re-read")

    sp = sub.add_parser("spin", help="drive motor(s) at target RPM for N seconds")
    sp.add_argument("--motor", choices=("left", "right", "both"), required=True)
    sp.add_argument("--rpm", type=int, required=True, help="signed target RPM")
    sp.add_argument("--duration", type=float, required=True, help="seconds (0, 30]")

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
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
    except ZLACDriverError as e:
        print(f"ZLAC driver error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
