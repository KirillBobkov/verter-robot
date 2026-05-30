"""ZLAC8015D Modbus register map — single source of truth.

═══════════════════════════════════════════════════════════════════════════════
TODO VERIFY — these addresses are typical for the ZLTECH ZLAC8015D family but
MAY vary between firmware revisions. Cross-check against the ZLAC8015D user
manual delivered with the unit BEFORE relying on this map in production.
Both `scripts/zlac_probe.py` and `zlac_driver.py` consume these constants.
═══════════════════════════════════════════════════════════════════════════════
"""
from __future__ import annotations

# Driver identification (R)
REG_DEVICE_ID = 0x0000          # product code / model
REG_FW_VERSION = 0x0001         # firmware version

# Operating mode (W, write once at setup)
REG_OPERATION_MODE = 0x200D
MODE_VELOCITY = 3               # closed-loop velocity control

# Control word (W) — enable / disable / fault clear / e-stop
REG_CONTROL_WORD = 0x200E
CTRL_ENABLE = 0x0008            # both channels enabled
CTRL_DISABLE = 0x0007           # both channels disabled
CTRL_CLEAR_FAULT = 0x0006       # clear latched fault flag
CTRL_EMERGENCY_STOP = 0x0005    # immediate hard stop

# Target velocity (W, signed int16 RPM, one register per motor)
REG_TARGET_RPM_M1 = 0x2088      # left motor
REG_TARGET_RPM_M2 = 0x2089      # right motor

# Actual velocity feedback (R, signed int16 RPM)
REG_ACTUAL_RPM_M1 = 0x20AB
REG_ACTUAL_RPM_M2 = 0x20AC

# Position counters (R, two registers high+low → int32 each)
REG_POSITION_M1_HI = 0x20A7
REG_POSITION_M1_LO = 0x20A8
REG_POSITION_M2_HI = 0x20A9
REG_POSITION_M2_LO = 0x20AA

# Driver status / fault (R)
REG_DRIVER_STATUS = 0x20A5      # status bit-field / running mode
REG_FAULT_CODE = 0x20A6         # 0 = no fault, else error code

# Comm-loss watchdog timeout (W, ms). Driver brakes if no Modbus traffic
# arrives for N ms. Hardware-level safety net on top of policy watchdog.
REG_COMM_WATCHDOG_MS = 0x2007
