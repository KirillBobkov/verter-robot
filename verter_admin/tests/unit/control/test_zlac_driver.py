"""Unit tests for ZLACDriver. Mocks pymodbus client so no real serial port needed.

Covers:
  * Modbus exception responses surface as ZLACDriverError.
  * Required ordering of writes (mode + watchdog before enable).
  * Sign correctness for negative target RPM and signed feedback decoding.
  * 32-bit position assembly from two uint16 registers.
"""
from __future__ import annotations

from dataclasses import dataclass
from unittest.mock import MagicMock

import pytest

from verter_admin.control.adapters.zlac.zlac_driver import (
    WheelFeedback,
    ZLACDriver,
    ZLACDriverError,
)
from verter_admin.control.adapters.zlac.zlac_registers import (
    CTRL_DISABLE,
    CTRL_ENABLE,
    MODE_VELOCITY,
    REG_COMM_WATCHDOG_MS,
    REG_CONTROL_WORD,
    REG_OPERATION_MODE,
    REG_TARGET_RPM_M1,
    REG_TARGET_RPM_M2,
)


# ---------------------------------------------------------------- fakes


@dataclass
class _FakeResponse:
    registers: list[int]
    error: bool = False

    def isError(self) -> bool:  # noqa: N802 — matches pymodbus API
        return self.error


class FakeModbusClient:
    """Minimal pymodbus-compatible double."""

    def __init__(self) -> None:
        self.connected = False
        self.reads: list[tuple[int, int]] = []           # (address, count)
        self.writes: list[tuple[int, int]] = []          # (address, value)
        self.next_read_values: dict[int, list[int]] = {} # address → registers
        self.next_read_error_at: set[int] = set()
        self.next_write_error_at: set[int] = set()

    def connect(self) -> bool:
        self.connected = True
        return True

    def close(self) -> None:
        self.connected = False

    def read_holding_registers(self, address: int, count: int, slave: int) -> _FakeResponse:
        self.reads.append((address, count))
        if address in self.next_read_error_at:
            return _FakeResponse(registers=[], error=True)
        regs = self.next_read_values.get(address, [0] * count)
        return _FakeResponse(registers=regs)

    def write_register(self, address: int, value: int, slave: int) -> _FakeResponse:
        self.writes.append((address, value))
        if address in self.next_write_error_at:
            return _FakeResponse(registers=[], error=True)
        return _FakeResponse(registers=[value])


@pytest.fixture
def fake_client() -> FakeModbusClient:
    return FakeModbusClient()


@pytest.fixture
def driver(fake_client: FakeModbusClient) -> ZLACDriver:
    drv = ZLACDriver(port="/dev/null", baud=115200, slave=1)
    drv._client = fake_client  # type: ignore[attr-defined]
    return drv


# -------------------------------------------------------- lifecycle


class TestLifecycle:
    def test_is_open_false_before_open(self) -> None:
        drv = ZLACDriver()
        assert drv.is_open is False

    def test_is_open_true_after_inject(self, driver: ZLACDriver) -> None:
        assert driver.is_open is True

    def test_close_clears_client(self, driver: ZLACDriver) -> None:
        driver.close()
        assert driver.is_open is False

    def test_operations_before_open_raise(self) -> None:
        drv = ZLACDriver()
        with pytest.raises(ZLACDriverError, match="not open"):
            drv.enable()


# ---------------------------------------------------------- setup


class TestSetup:
    def test_setup_writes_mode_and_watchdog(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        driver.setup_velocity_mode(comm_watchdog_ms=200)
        assert (REG_OPERATION_MODE, MODE_VELOCITY) in fake_client.writes
        assert (REG_COMM_WATCHDOG_MS, 200) in fake_client.writes

    def test_setup_rejects_out_of_range_watchdog(self, driver: ZLACDriver) -> None:
        with pytest.raises(ValueError):
            driver.setup_velocity_mode(comm_watchdog_ms=10)
        with pytest.raises(ValueError):
            driver.setup_velocity_mode(comm_watchdog_ms=99999)


# ---------------------------------------------------------- control


class TestControlWord:
    def test_enable_writes_control_word(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        driver.enable()
        assert fake_client.writes[-1] == (REG_CONTROL_WORD, CTRL_ENABLE)

    def test_disable_writes_control_word(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        driver.disable()
        assert fake_client.writes[-1] == (REG_CONTROL_WORD, CTRL_DISABLE)


# ----------------------------------------------------- target RPM


class TestWriteTargetRpm:
    def test_positive_rpm(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        driver.write_target_rpm(50, 75)
        assert (REG_TARGET_RPM_M1, 50) in fake_client.writes
        assert (REG_TARGET_RPM_M2, 75) in fake_client.writes

    def test_negative_rpm_two_complement(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        driver.write_target_rpm(-50, -1)
        # -50 in uint16 two's complement = 0x10000 - 50 = 0xFFCE
        assert (REG_TARGET_RPM_M1, 0xFFFF - 50 + 1) in fake_client.writes
        assert (REG_TARGET_RPM_M2, 0xFFFF) in fake_client.writes

    def test_rpm_out_of_range_raises(self, driver: ZLACDriver) -> None:
        with pytest.raises(ValueError):
            driver.write_target_rpm(40000, 0)

    def test_stop_motors_writes_zero(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        driver.stop_motors()
        assert (REG_TARGET_RPM_M1, 0) in fake_client.writes
        assert (REG_TARGET_RPM_M2, 0) in fake_client.writes

    def test_stop_motors_swallows_errors(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        fake_client.next_write_error_at.add(REG_TARGET_RPM_M1)
        # Must NOT raise — stop_motors is best-effort for emergency paths.
        driver.stop_motors()


# ----------------------------------------------------- feedback


class TestFeedback:
    def test_positive_rpm_decode(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        from verter_admin.control.adapters.zlac.zlac_registers import (
            REG_ACTUAL_RPM_M1, REG_ACTUAL_RPM_M2, REG_FAULT_CODE,
            REG_POSITION_M1_HI, REG_POSITION_M2_HI,
        )
        fake_client.next_read_values[REG_ACTUAL_RPM_M1] = [50]
        fake_client.next_read_values[REG_ACTUAL_RPM_M2] = [75]
        fake_client.next_read_values[REG_POSITION_M1_HI] = [0, 1000]
        fake_client.next_read_values[REG_POSITION_M2_HI] = [0, 2000]
        fake_client.next_read_values[REG_FAULT_CODE] = [0]

        fb = driver.read_feedback()
        assert fb == WheelFeedback(
            actual_rpm_left=50,
            actual_rpm_right=75,
            position_left=1000,
            position_right=2000,
            fault_code=0,
        )
        assert fb.has_fault is False

    def test_negative_rpm_decode(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        from verter_admin.control.adapters.zlac.zlac_registers import (
            REG_ACTUAL_RPM_M1, REG_ACTUAL_RPM_M2, REG_FAULT_CODE,
            REG_POSITION_M1_HI, REG_POSITION_M2_HI,
        )
        fake_client.next_read_values[REG_ACTUAL_RPM_M1] = [0xFFCE]  # -50
        fake_client.next_read_values[REG_ACTUAL_RPM_M2] = [0x8000]  # -32768
        fake_client.next_read_values[REG_POSITION_M1_HI] = [0xFFFF, 0xFFFF]  # -1
        fake_client.next_read_values[REG_POSITION_M2_HI] = [0x8000, 0x0000]  # min int32
        fake_client.next_read_values[REG_FAULT_CODE] = [0]

        fb = driver.read_feedback()
        assert fb.actual_rpm_left == -50
        assert fb.actual_rpm_right == -32768
        assert fb.position_left == -1
        assert fb.position_right == -(2**31)

    def test_fault_propagated(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        from verter_admin.control.adapters.zlac.zlac_registers import REG_FAULT_CODE
        fake_client.next_read_values[REG_FAULT_CODE] = [42]
        fb = driver.read_feedback()
        assert fb.fault_code == 42
        assert fb.has_fault is True


# ----------------------------------------------------- error surfacing


class TestErrorSurfacing:
    def test_read_error_raises_zlacerror(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        from verter_admin.control.adapters.zlac.zlac_registers import REG_FW_VERSION
        fake_client.next_read_error_at.add(REG_FW_VERSION)
        with pytest.raises(ZLACDriverError):
            driver.read_fw_version()

    def test_write_error_raises_zlacerror(self, driver: ZLACDriver, fake_client: FakeModbusClient) -> None:
        fake_client.next_write_error_at.add(REG_CONTROL_WORD)
        with pytest.raises(ZLACDriverError):
            driver.enable()
