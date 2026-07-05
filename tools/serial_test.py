import serial, time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# 先主动拉低 DTR，释放 STM32 复位
ser.dtr = False
time.sleep(2)

# 发版本查询: 55 55 02 01
ser.write(b'\x55\x55\x02\x01')
print("已发送: 55 55 02 01")

# 读回复，期望 6 字节
rx = ser.read(6)
print(f"收到 {len(rx)} 字节: {' '.join(f'{b:02X}' for b in rx)}")

ser.close()
