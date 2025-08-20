import serial
import time
import pygame

# 시리얼 포트 설정 (윈도우에서는 COM 포트 확인 필요, 예: COM3)
ser = serial.Serial('COM13', 9600, timeout=1)

# 조이스틱 초기화
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
def create_byte_command(angle, speed):
    """
    angle: -26 ~ 26
    speed: 0 (정지) or 1 (전진)
    """
    # 6비트로 매핑: -26 ~ 26 → 0~52
    angle6bit = angle + 26
    if angle6bit < 0: angle6bit = 0
    if angle6bit > 63: angle6bit = 63

    # 1바이트 생성
    byte_val = (speed << 6) | angle6bit

    # 패리티 비트 생성 (홀수 패리티)
    parity = bin(byte_val).count('1') % 2
    byte_val |= (parity << 7)

    return byte_val 
while True:
    pygame.event.pump()
    x_axis = joystick.get_axis(0)  # 조향
    y_axis = joystick.get_axis(1)  # 속도

    # -1~1 범위를 0~255 범위로 맵핑
    steering = int((x_axis + 1) * 127.5)
    throttle = int(((-y_axis) + 1) * 127.5)  # 보통 -1이 전진이라 반전

   
    ser.write(create_byte_command(steering,throttle))
    time.sleep(0.05)
