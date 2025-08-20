import serial
import time
import pygame

# 시리얼 포트 설정
ser = serial.Serial('COM4', 9600, timeout=1)

# 조이스틱 초기화
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()


def create_byte_command(angle, speed):
    """
    angle: -26 ~ 26
    speed: -1, 0, 1
    """
    # 6비트 매핑
    angle6bit = angle + 26
    if angle6bit < 0: angle6bit = 0
    if angle6bit > 63: angle6bit = 63

    # 2비트 속도
    if speed == 1:
        speed_bits = 0b01  # 전진
    elif speed == -1:
        speed_bits = 0b10  # 후진
    else:
        speed_bits = 0b00  # 정지

    # 1바이트 생성
    byte_val = (speed_bits << 6) | angle6bit
    return byte_val


# 데드존 처리 함수
def apply_deadzone(value, deadzone=0.1):
    if abs(value) < deadzone:
        return 0.0
    return value

while True:
    pygame.event.pump()
    if joystick.get_button(0):  # B 버튼
        ser.write(b'K')  # 문자열 'K' 한 바이트 전송
        
    # 스틱 축 읽기
    x_axis = apply_deadzone(joystick.get_axis(1))  # 조향
    y_axis = apply_deadzone(joystick.get_axis(2))  # 속도
    # print(x_axis)
    # print(y_axis)
    # steering: -26 ~ 26
    steering = -int(y_axis * 26)

    # throttle: -1, 0, 1
    if x_axis < 0.0:
        throttle = 1   # 전진
    elif x_axis > 0.0:
        throttle = -1  # 후진
    elif x_axis ==0:
        throttle = 0   # 정지

    # 시리얼 전송
    # print(steering)

    cmd = create_byte_command(steering, throttle)
    ser.write(bytes([cmd]))

    print(f"Steering: {steering}, Throttle: {throttle}")
    time.sleep(0.05)
