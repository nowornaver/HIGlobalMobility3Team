import serial
import keyboard
import time

SERIAL_PORT = 'COM12'
SERIAL_BAUD = 9600
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

print("WASD로 제어, ESC로 종료")

while True:
    if keyboard.is_pressed('esc'):
        break

    key_to_send = None

    if keyboard.is_pressed('w'):
        key_to_send = 'w'
    elif keyboard.is_pressed('s'):
        key_to_send = 's'
    elif keyboard.is_pressed('a'):
        key_to_send = 'a'
    elif keyboard.is_pressed('d'):
        key_to_send = 'd'

    if key_to_send:
        ser.write(key_to_send.encode('utf-8'))
        print(f"전송: {key_to_send}")

    time.sleep(0.1)
