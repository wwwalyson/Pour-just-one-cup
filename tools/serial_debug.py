#!/usr/bin/env python3
"""串口调试 v2 — 同时控制 DTR 和 RTS"""
import serial, time, os, stat, termios, fcntl

PORT = '/dev/ttyUSB0'
BAUD = 9600

def get_lines(ser):
    flags = int.from_bytes(fcntl.ioctl(ser.fd, termios.TIOCMGET, bytes(4)), 'little')
    return {'DTR': bool(flags & termios.TIOCM_DTR),
            'RTS': bool(flags & termios.TIOCM_RTS)}

print("设备类型:", '字符设备' if stat.S_ISCHR(os.stat(PORT).st_mode) else '异常')

# === 方案A: 先 RTS LOW, 再 DTR LOW (确保正常启动非bootloader) ===
print("\n=== 方案A: RTS先拉低 → DTR再拉低 (正常启动) ===")
ser = serial.Serial(PORT, BAUD, timeout=1)
print("打开后:", get_lines(ser))

ser.rts = False
time.sleep(0.1)
print("RTS↓后:", get_lines(ser))

ser.dtr = False
time.sleep(0.1)
print("DTR↓后:", get_lines(ser))

print("等待启动...", end=' ', flush=True)
time.sleep(2)
print("done")

ser.write(b'\x55\x55\x02\x01')
time.sleep(0.3)
rx = ser.read(32)
print(f"回复: {len(rx)} 字节 -> {' '.join(f'{b:02X}' for b in rx) if rx else '(空)'}")
ser.close()
time.sleep(0.5)

# === 方案B: 直接用 ioctl 同时清除 DTR+RTS ===
print("\n=== 方案B: ioctl 同时清除 DTR+RTS ===")
ser = serial.Serial(PORT, BAUD, timeout=1)
TIOCMBIC = termios.TIOCMBIC
TIOCM_DTR = termios.TIOCM_DTR
TIOCM_RTS = termios.TIOCM_RTS
fcntl.ioctl(ser.fd, TIOCMBIC, TIOCM_DTR | TIOCM_RTS)
print("清除后:", get_lines(ser))
time.sleep(2)

ser.write(b'\x55\x55\x02\x01')
time.sleep(0.3)
rx = ser.read(32)
print(f"回复: {len(rx)} 字节 -> {' '.join(f'{b:02X}' for b in rx) if rx else '(空)'}")
ser.close()
time.sleep(0.5)

# === 方案C: 纯 RTS 控制 (验证板子是否实际用RTS复位) ===
print("\n=== 方案C: 只拉低RTS, DTR不动 ===")
ser = serial.Serial(PORT, BAUD, timeout=1)
print("打开后:", get_lines(ser))

ser.rts = False
time.sleep(0.1)
print("RTS↓后:", get_lines(ser))

print("等待启动...", end=' ', flush=True)
time.sleep(2)
print("done")

ser.write(b'\x55\x55\x02\x01')
time.sleep(0.3)
rx = ser.read(32)
print(f"回复: {len(rx)} 字节 -> {' '.join(f'{b:02X}' for b in rx) if rx else '(空)'}")
ser.close()

print("\n请观察以上哪种方案能让舵机发出响声（说明STM32正常启动）")
print("以及哪种方案能收到回复数据")
