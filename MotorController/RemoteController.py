import serial
import msvcrt

SERIAL_PORT = 'COM3'
SERIAL_BAUD = 115200
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
Steering_angle = 0
print("아무 키나 눌러보세요. ESC 키를 누르면 종료됩니다.")
while True:

    if msvcrt.kbhit():  # 키보드 입력이 있으면
        key = msvcrt.getch()  # 1글자 읽기 (바이트형)
        print(f"입력한 키: {key}")

        if key == b'\x1b':  # ESC 키(ASCII 27)
            break
    if (Steering_angle <-16) :
        Steering_angle = 0
    elif(Steering_angle >21) :
        Steering_angle = 0 
    if (key == 'w') :
        state = "FORWARD"
    elif(key == 'a') :
        state = "TURNLEFT"
        Steering_angle = Steering_angle-1
    

    elif (key == 'd') :
        state ="TURNRIGHT"
        Steering_angle=Steering_angle+1

    elif(key == 's') :
        state = "REVERSING"



    cmd_str = f"{state},{Steering_angle}\n"
    ser.write(cmd_str.encode('utf-8'))


