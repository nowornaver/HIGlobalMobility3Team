import serial
import msvcrt

SERIAL_PORT = 'COM19'
SERIAL_BAUD = 115200
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

Steering_angle = 0
state = "STOP"
key = b""

print("아무 키나 눌러보세요. ESC 키를 누르면 종료됩니다.")

while True:
    if msvcrt.kbhit():  # 키보드 입력이 있으면
        key = msvcrt.getch()  # 1글자 읽기 (바이트형)
        print(f"입력한 키: {key}")

        if key == b'\x1b':  # ESC 키(ASCII 27)
            break

        # 키 처리
        if key == b'w':
            state = "FORWARD"
        elif key == b'a':
            state = "TURNLEFT"
            Steering_angle -= 1
        elif key == b'd':
            state = "TURNRIGHT"
            Steering_angle += 1
        elif key == b's':
            state = "REVERSING"
        else:
            state = "STOP"

        # 조향각 제한
        if Steering_angle < -16:
            Steering_angle = -16
        elif Steering_angle > 21:
            Steering_angle = 21

        # 시리얼 전송
        cmd_str = f"{state},{Steering_angle}\n"
        ser.write(cmd_str.encode('utf-8'))
        print(f"전송: {cmd_str.strip()}")
