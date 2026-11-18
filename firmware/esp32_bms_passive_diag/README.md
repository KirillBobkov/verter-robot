# Passive BMS UART diagnostics

This separate PlatformIO firmware has no BMS transmit function. GPIO16 and GPIO17
are inputs; only the selected RX is connected to UART2. USB commands configure the
local receiver: `BEGIN <16|17> <baud>`, `END`, `STATUS`. No ROS or motor interface.

UART: 8N1, no parity check, no hardware flow control. Supported baud rates:
1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800.

`STAT` reports received bytes, zero bytes, UART error events, software samples of
RX pin level, occurrences of 4E57 and 55AAEB90 prefixes, and the first 256 bytes.
A matching prefix alone is not a valid packet. Full traffic is not retained;
only the prefix scan covers all consumed bytes. Samples cannot rule out other
protocols. Event counters can undercount if the event queue saturates; pin-level
samples are not a waveform or calibrated duty-cycle measurement. Parity errors
are not meaningful with parity disabled. FIFO/buffer counters describe the UART
receiver; serial output is deliberately limited to reduce logging overhead.

Build this diagnostic firmware with PlatformIO. Before any manual upload, identify
the board and its port explicitly; do not rely on a fixed /dev/ttyUSB number.
Save and verify the correct full-flash backup before replacing firmware.
Firmware changes and probing require user authorization; never run this against
an unidentified board or a chassis/IMU port. CH340 by-id strings are non-unique.
