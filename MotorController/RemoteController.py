import serial
import keyboard  # pip install keyboard
import time
SERIAL_PORT = 'COM9'
SERIAL_BAUD = 115200
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

Steering_angle = 0
state = "STOP"

print("WASD로 제어, ESC로 종료")

while True:
    if keyboard.is_pressed('esc'):
        break

    if keyboard.is_pressed('w'):
        state = "FORWARD"
    elif keyboard.is_pressed('s'):
        state = "REVERSING"
    else:
        state = "STOP"

    if keyboard.is_pressed('a'):
        Steering_angle -= 1
    elif keyboard.is_pressed('d'):
        Steering_angle += 1

    # 조향각 제한
    if Steering_angle < -16:
        Steering_angle = -16
    elif Steering_angle > 21:
        Steering_angle = 21

    # 시리얼 전송
    cmd_str = f"{state},{Steering_angle}\n"
    ser.write(cmd_str.encode('utf-8'))
    print(f"전송: {cmd_str.strip()}")

    # 0.1초마다 전송
    time.sleep(0.1)
