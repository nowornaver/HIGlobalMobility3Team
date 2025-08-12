import serial, keyboard, time

ser = serial.Serial('COM3', 115200, timeout=1)

Steering_angle = 0
state = "STOP"
prev_state = None
prev_angle = None   # ← 이전에 보낸 각도 저장

print("WASD로 제어, ESC로 종료")

while True:
    if keyboard.is_pressed('esc'):
        break

    # 1) 상태 판정과 각도 조작
    if keyboard.is_pressed('w'):
        state = "FORWARD"
        Steering_angle = 0        # ← 전진 시엔 0도로 고정
    elif keyboard.is_pressed('s'):
        state = "REVERSING"
        Steering_angle = 0        # ← 후진 시에도 0도로 고정
    elif keyboard.is_pressed('a'):
        state = "TURNLEFT"
        Steering_angle = max(Steering_angle - 1, -16)
    elif keyboard.is_pressed('d'):
        state = "TURNRIGHT"
        Steering_angle = min(Steering_angle + 1, 21)
    else:
        state = "STOP"
        # (원하면 STOP 시에도 Steering_angle = 0)

    # 2) 상태 또는 각도가 변경되었을 때만 전송
    if state != prev_state or Steering_angle != prev_angle:
        cmd_str = f"{state},{Steering_angle}\n"
        ser.write(cmd_str.encode('utf-8'))
        print("전송:", cmd_str.strip())
        prev_state = state
        prev_angle = Steering_angle

    time.sleep(0.1)
