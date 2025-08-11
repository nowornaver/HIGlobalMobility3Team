import serial
import keyboard
import time

SERIAL_PORT = 'COM12'
SERIAL_BAUD = 9600
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
angle = 5
print("WASD로 제어, ESC로 종료")

while True:
    ser.write(angle.to_bytes(1, byteorder='little', signed=True))


    time.sleep(0.1)
