"""ZLAC8015D Modbus driver — sole protocol adapter for the chassis.

Wraps pymodbus with a clean Python API used by the ROS lifecycle node. This is
the only module in the chassis stack that imports pymodbus (SDHR rulebook §2.2:
adapters are the boundary where protocol glue lives).

The driver is **stateless with respect to robot behavior** — all lifecycle and
policy decisions belong to ChassisPolicy. The driver simply exposes:
  * connection lifecycle (open/close),
  * one-time setup (velocity mode, comm-loss watchdog),
  * runtime ops (enable/disable, write targets, read feedback),
  * fault read/clear.

Errors from pymodbus surface as `ZLACDriverError` for callers to catch
uniformly.
"""
from __future__ import annotations

from dataclasses import dataclass

from .zlac_registers import (
    CTRL_CLEAR_FAULT,
    CTRL_DISABLE,
    CTRL_ENABLE,
    CTRL_EMERGENCY_STOP,
    MODE_VELOCITY,
    REG_ACTUAL_RPM_M1,
    REG_ACTUAL_RPM_M2,
    REG_COMM_WATCHDOG_MS,
    REG_CONTROL_WORD,
    REG_DRIVER_STATUS,
    REG_FAULT_CODE,
    REG_FW_VERSION,
    REG_OPERATION_MODE,
    REG_POSITION_M1_HI,
    REG_POSITION_M2_HI,
    REG_TARGET_RPM_M1,
    REG_TARGET_RPM_M2,
)


class ZLACDriverError(RuntimeError):
    """Raised on any Modbus failure (timeout, CRC, exception response, etc.)."""


@dataclass(frozen=True)
class WheelFeedback:
    """Single snapshot of motor feedback values read from the driver."""

    actual_rpm_left: int
    actual_rpm_right: int
    position_left: int
    position_right: int
    fault_code: int

    @property
    def has_fault(self) -> bool:
        return self.fault_code != 0


def _to_signed16(raw: int) -> int:
    return raw - 0x10000 if raw & 0x8000 else raw


def _to_uint16(value: int) -> int:
    if value < -0x8000 or value > 0x7FFF:
        raise ValueError(f"value {value} outside int16 range")
    return value & 0xFFFF


def _combine_int32(hi: int, lo: int) -> int:
    raw = (hi << 16) | lo
    return raw - 0x100000000 if raw & 0x80000000 else raw


class ZLACDriver:
    """ZLAC8015D Modbus RTU adapter.

    Parameters mirror typical bench/probe defaults. The constructor only stores
    them; `open()` actually establishes the serial connection.
    """

    def __init__(
        self,
        port: str = "/dev/zlac_chassis",
        baud: int = 115200,
        slave: int = 1,
        timeout: float = 0.2,
    ) -> None:
        self._port = port
        self._baud = baud
        self._slave = slave
        self._timeout = timeout
        self._client = None  # type: ignore[assignment]

    # ------------------------------------------------------ lifecycle

    def open(self) -> None:
        # Defer pymodbus import so unit tests / dev machines without pymodbus
        # can still import this module (e.g. for type references).
        try:
            from pymodbus.client import ModbusSerialClient
        except ImportError as e:
            raise ZLACDriverError(
                "pymodbus not installed; pip install 'pymodbus>=3.0' pyserial"
            ) from e

        self._client = ModbusSerialClient(
            port=self._port,
            baudrate=self._baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self._timeout,
        )
        if not self._client.connect():
            self._client = None
            raise ZLACDriverError(f"failed to open serial port {self._port}")

    def close(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            finally:
                self._client = None

    @property
    def is_open(self) -> bool:
        return self._client is not None

    # ----------------------------------------------------- low-level I/O

    def _read(self, address: int, count: int = 1) -> list[int]:
        self._require_open()
        result = self._client.read_holding_registers(
            address=address, count=count, slave=self._slave
        )
        if result.isError():
            raise ZLACDriverError(
                f"read {count}@0x{address:04X} failed: {result}"
            )
        return list(result.registers)

    def _write(self, address: int, value: int) -> None:
        self._require_open()
        result = self._client.write_register(
            address=address, value=value, slave=self._slave
        )
        if result.isError():
            raise ZLACDriverError(
                f"write 0x{address:04X}=0x{value:04X} failed: {result}"
            )

    def _require_open(self) -> None:
        if self._client is None:
            raise ZLACDriverError("driver not open — call open() first")

    # ---------------------------------------------------------- setup

    def setup_velocity_mode(self, comm_watchdog_ms: int = 200) -> None:
        """One-time setup before enabling drives.

        * Selects closed-loop velocity control.
        * Configures hardware comm-loss watchdog — if no Modbus traffic for
          `comm_watchdog_ms` the driver brakes automatically. This is the
          hardware safety net on top of the ChassisPolicy software watchdog.
        """
        if comm_watchdog_ms < 50 or comm_watchdog_ms > 5000:
            raise ValueError("comm_watchdog_ms outside [50, 5000]")
        self._write(REG_OPERATION_MODE, MODE_VELOCITY)
        self._write(REG_COMM_WATCHDOG_MS, comm_watchdog_ms)

    def read_fw_version(self) -> int:
        return self._read(REG_FW_VERSION)[0]

    # ------------------------------------------------------ runtime ops

    def enable(self) -> None:
        self._write(REG_CONTROL_WORD, CTRL_ENABLE)

    def disable(self) -> None:
        self._write(REG_CONTROL_WORD, CTRL_DISABLE)

    def emergency_stop(self) -> None:
        self._write(REG_CONTROL_WORD, CTRL_EMERGENCY_STOP)

    def clear_fault(self) -> None:
        self._write(REG_CONTROL_WORD, CTRL_CLEAR_FAULT)

    def write_target_rpm(self, left_rpm: int, right_rpm: int) -> None:
        """Set target RPM for both motors. Signed int16 per channel."""
        self._write(REG_TARGET_RPM_M1, _to_uint16(left_rpm))
        self._write(REG_TARGET_RPM_M2, _to_uint16(right_rpm))

    def stop_motors(self) -> None:
        """Best-effort zero target — used in safe-stop / shutdown."""
        try:
            self.write_target_rpm(0, 0)
        except ZLACDriverError:
            pass

    # ------------------------------------------------------- feedback

    def read_feedback(self) -> WheelFeedback:
        """Snapshot of actual RPM + position + fault for both motors.

        Uses 4 separate Modbus reads (RPM L/R, position L, position R, fault).
        At 115200 baud each round-trip is ≈ 4 ms → full snapshot ≈ 16-20 ms,
        leaving budget for write_target_rpm + spin at 50 Hz.
        """
        rpm_l = _to_signed16(self._read(REG_ACTUAL_RPM_M1)[0])
        rpm_r = _to_signed16(self._read(REG_ACTUAL_RPM_M2)[0])
        pos_l_regs = self._read(REG_POSITION_M1_HI, count=2)
        pos_r_regs = self._read(REG_POSITION_M2_HI, count=2)
        fault = self._read(REG_FAULT_CODE)[0]
        return WheelFeedback(
            actual_rpm_left=rpm_l,
            actual_rpm_right=rpm_r,
            position_left=_combine_int32(pos_l_regs[0], pos_l_regs[1]),
            position_right=_combine_int32(pos_r_regs[0], pos_r_regs[1]),
            fault_code=fault,
        )

    def read_fault_only(self) -> int:
        """Cheap single-register read for fault-only polling."""
        return self._read(REG_FAULT_CODE)[0]

    def read_status(self) -> int:
        return self._read(REG_DRIVER_STATUS)[0]

    def read_register(self, address: int) -> int:
        """Public single-register read — used by the bench probe CLI for ad-hoc
        debugging at arbitrary addresses."""
        return self._read(address)[0]
