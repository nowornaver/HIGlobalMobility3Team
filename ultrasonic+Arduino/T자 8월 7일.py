# 웨이포인트 기반 T자 주차 
# 웨이포인트를 따라가다가 직각주차 시작 구간의 위도,경도값 또는 웨이포인트의 인덱스를 미리 체크하고
# 그 지점에서 이 코드가 실행 되도록 통합하는 방향으로 설계됨
# 코드가 실행되면 차량 양 옆에 있는 초음파 센서가 활성화 되며 주차구간에 진입하게 되고 전진하다가 끝 지점에 도달하면 정지 후 후진으로 변경
# 변경되는 시점에서부터 왼쪽에 있는 센서가 100cm라는 일정 거리를 유지하면서 일정거리에서 멀어지거나 가까워지게 되면 조향각을 +-1도 씩 수정
# 지속적으로 후진 하다가 후방 중앙에 있는 센서를 통해 후진중인 차량의 뒤에 있는 연석을 감지하여 일정 값을 받으면 정지
# 나가는 구간도 오른쪽 센서가 100cm를 유지한 체 웨이포인트를 따라서 주행

import serial
import time
import threading
import csv
import sys
from datetime import datetime

# 전역 설정
TARGET_DISTANCE = 100        # 목표 간격 (cm)
TOLERANCE = 10               # 허용 오차 범위 ±10cm
CSV_FILENAME = "spacing_log.csv"

# 시리얼 포트 설정 (필요 시 COM 포트 수정)
serial_port = serial.Serial('COM8', 115200, timeout=1)
time.sleep(2)

# 현재 기준 센서 모드 (0=왼쪽, 1=오른쪽) 전진과 후진을 구분하여 어떤 센서 값을 받아올지 결정
# 실제 주행에서는 엔코더 값 받을 예정
sensor_mode = 0

# 입력 스레드: 센서 기준 실시간 전환
def input_thread():
    global sensor_mode
    while True:
        user_input = input("\n센서 기준 변경 (0=왼쪽, 1=오른쪽): ").strip()
        if user_input == "0":
            sensor_mode = 0
            print("🔄 왼쪽 센서 기준으로 전환됨")
        elif user_input == "1":
            sensor_mode = 1
            print("🔄 오른쪽 센서 기준으로 전환됨")

# 스레드 시작
threading.Thread(target=input_thread, daemon=True).start()

# 센서 값 읽기
def get_distances():
    while serial_port.in_waiting:
        line = serial_port.readline().decode('utf-8').strip()
    try:
        _, left, right = line.split(",")
        return float(left), float(right)
    except:
        return None, None

# CSV 파일 초기화 (시각화를 위한 CSV파일 생성) 
with open(CSV_FILENAME, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "Left(cm)", "Right(cm)", "Mode", "Deviation"])

# 메인 루프
print("🚗 간격 유지 주행 시작 (Ctrl+C로 종료)\n")
try:
    while True:
        left, right = get_distances()
        if left is None or right is None:
            continue

        if sensor_mode == 0:
            distance = left
            label = "왼쪽"
        else:
            distance = right
            label = "오른쪽"

        deviation = distance - TARGET_DISTANCE

        # 간격 유지 상태 메시지
        if abs(deviation) <= TOLERANCE:
            status = "✅ 간격 유지 중"
        elif deviation > 0:
            status = f"⬅️ 너무 멀음 ({int(deviation)}cm)"
        else:
            status = f"➡️ 너무 가까움 ({int(-deviation)}cm)"

        # 출력
        sys.stdout.write(f"\r📏 왼쪽: {left:5.1f} cm | 오른쪽: {right:5.1f} cm | 기준: {label:<4} | {status:<20}")
        sys.stdout.flush()

        # CSV 저장
        with open(CSV_FILENAME, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([datetime.now().isoformat(), left, right, label, round(deviation, 1)])

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n🛑 주행 종료")
    serial_port.close()
