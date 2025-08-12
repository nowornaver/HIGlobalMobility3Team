import serial
import time

SERIAL_PORT = 'COM13'
SERIAL_BAUD = 9600
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

print("Sending 'C' mode, then sending numbers repeatedly")

# 먼저 모드 문자 한 번 보내기
ser.write(b'C')

while True:
    # 예: 숫자 '1' 문자 계속 보내기 (아두이노에서 '1' 문자 받음)
    ser.write(b'1')  
    time.sleep(0.1)
