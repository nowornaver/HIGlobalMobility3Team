import serial
import time
import pygame

# 시리얼 포트 설정
ser = serial.Serial('COM13', 9600, timeout=1)

# 조이스틱 초기화
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()


# 데드존 처리 함수
def apply_deadzone(value, deadzone=0.1):
    if abs(value) < deadzone:
        return 0.0
    return value

seq = 0  # 순서 번호

while True:
    pygame.event.pump()

    # 모드 전환 버튼 (예: B 버튼)
    if joystick.get_button(0):
        ser.write(b'K')  # 모드 전환용 한 바이트 전송

    # 스틱 축 읽기
    x_axis = apply_deadzone(joystick.get_axis(1))  # 조향
    y_axis = apply_deadzone(joystick.get_axis(2))  # 속도

    # steering: -26 ~ 26 범위
    steering = -int(y_axis * 26)

    # throttle: -1, 0, 1
    if x_axis < 0.0:
        throttle = 5   # 전진
    elif x_axis > 0.0:
        throttle = -5  # 후진
    else:
        throttle = 0   # 정지

    # ===============================
    # 속도와 조향을 "각각 1바이트"로 전송
    # ===============================
    # ser.write(bytes([seq & 0xFF]))

    ser.write(bytes([throttle & 0xFF]))   # 속도 (signed 8bit)
    ser.write(bytes([steering & 0xFF]))   # 조향 (signed 8bit)

    # print(f"Steering: {steering}, Throttle: {throttle}")
    print(f"Seq: {seq}, Throttle: {throttle}, Steering: {steering}")
    # seq = (seq + 1) % 256  # 0~255 반복
    time.sleep(0.1)  # 100ms 간격
