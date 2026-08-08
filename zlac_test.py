import serial

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

import time
import sys

PORT = '/dev/chassis'
BAUDRATE = 115200

#READ_HOLDING_REGISTERS = 3

ser = serial.Serial(port=PORT, baudrate=BAUDRATE, bytesize=8, parity='N', stopbits=1, xonxoff=0)

print('Serial opened...')
time.sleep(1)

master = modbus_rtu.RtuMaster(ser)
master.set_timeout(0.1)
master.set_verbose(True)
print('Modbus master created...')
time.sleep(1)



res = master.execute(1, cst.WRITE_SINGLE_REGISTER, 0x200D, output_value=3)
print(f'Set velocity mode: {res}')

res = master.execute(1, cst.WRITE_SINGLE_REGISTER, 0x2000, output_value=0)
print(f'Set watchdog ms: {res}')
time.sleep(1)


res = master.execute(1, cst.READ_HOLDING_REGISTERS, 0x20A5, 8)
print(res)

master.close()
ser.close()
sys.exit()

res = master.execute(1, cst.WRITE_SINGLE_REGISTER, 0x200E, output_value=0x0008)
print(f'Driver enable: {res}')
time.sleep(1)


TARGET_RPM = 1
# Вперед
#res = master.execute(1, cst.WRITE_MULTIPLE_REGISTERS, 0x2088, output_value=[TARGET_RPM, 65536 - TARGET_RPM])
# Назад
res = master.execute(1, cst.WRITE_MULTIPLE_REGISTERS, 0x2088, output_value=[65536 - TARGET_RPM, TARGET_RPM])
print(f'Set RPM: [{TARGET_RPM}] -> {res}')

time.sleep(2)

res = master.execute(1, cst.WRITE_MULTIPLE_REGISTERS, 0x2088, output_value=[0, 0])
print(f'Set RPM: [0] -> {res}')