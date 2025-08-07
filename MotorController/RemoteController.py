import serial
import keyboard  # pip install keyboard
import time

SERIAL_PORT = 'COM9'
SERIAL_BAUD = 115200
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

Steering_angle = 0
state = "STOP"

print("WASD로 제어, ESC로 종료")

prev_state = "STOP"  # 이전 상태 저장

while True:
    if keyboard.is_pressed('esc'):
        break

    # 기본 상태는 STOP
    state = "STOP"

    # 전후진
    if keyboard.is_pressed('w'):
        state = "FORWARD"
    elif keyboard.is_pressed('s'):
        state = "REVERSING"

    # 좌우 조향
    if keyboard.is_pressed('a'):
        state = "TURNLEFT"
        Steering_angle -= 1
    elif keyboard.is_pressed('d'):
        state = "TURNRIGHT"
        Steering_angle += 1

    # 조향각 제한
    if Steering_angle < -16:
        Steering_angle = -16
    elif Steering_angle > 21:
        Steering_angle = 21

    # 상태가 변했을 때만 전송 (불필요한 중복 전송 방지)
    if state != prev_state or state != "STOP":
        cmd_str = f"{state},{Steering_angle}\n"
        ser.write(cmd_str.encode('utf-8'))
        print(f"전송: {cmd_str.strip()}")
        prev_state = state

    time.sleep(0.1)
