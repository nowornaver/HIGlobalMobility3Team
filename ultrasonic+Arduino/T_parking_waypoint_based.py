import serial
import time
import sys

# 시리얼 포트 설정
serial_port = serial.Serial('COM9', 115200, timeout=1)
time.sleep(2)

# 거리 기준 (cm)
STOP_THRESHOLD = 50
RECOVERY_DISTANCE = 100
FRONT_COLLISION_LIMIT = 50
SAFE_FRONT_MARGIN = 20  # 전방 안전 거리 여유

# 감지 횟수 기준
STOP_COUNT_REQUIRED = 5
RECOVERY_COUNT_REQUIRED = 5
FRONT_SAFE_COUNT_REQUIRED = 3

# 상태 정의
state = "REVERSING"
rear_stop_count = 0
rear_clear_count = 0
front_safe_count = 0

def get_distances():
    line = None
    while serial_port.in_waiting:
        line = serial_port.readline().decode('utf-8').strip()

    if line:
        try:
            _, front, rear = line.split(",")
            return float(front), float(rear)
        except ValueError:
            print("거리 파싱 실패:", line)
    return None, None

def get_steering_angle():
    try:
        angle = float(input("\n🎮 조향각 입력 (0도일 때만 전진 허용): "))
        return angle
    except ValueError:
        print("⚠️ 잘못된 입력")
        return None

def print_status(front, rear, state, count):
    sys.stdout.write(
        f"\r📏 전방: {front:6.1f} cm | 후방: {rear:6.1f} cm | 상태: {state:<18} | 감지 카운트: {count}"
    )
    sys.stdout.flush()

def main():
    global state, rear_stop_count, rear_clear_count, front_safe_count

    while True:
        front, rear = get_distances()
        if front is None or rear is None:
            continue

        print_status(front, rear, state, rear_stop_count)

        if state == "REVERSING":
            if rear <= STOP_THRESHOLD:
                rear_stop_count += 1
            else:
                rear_stop_count = 0

            if rear_stop_count >= STOP_COUNT_REQUIRED:
                print("\n🛑 차량 정지 (후방 장애물 연속 감지)")
                state = "STEER_RESET"
                rear_stop_count = 0

        elif state == "STEER_RESET":
            print("\n↔️ 조향각 리셋 대기 중...")
            angle = None
            while angle is None or abs(angle) > 1:
                angle = get_steering_angle()
                if angle is not None and abs(angle) <= 1:
                    print("✅ 조향 리셋 완료 → 전진 시작")
                    state = "FORWARD"
                    rear_clear_count = 0
                    break

        elif state == "FORWARD":
            if front <= FRONT_COLLISION_LIMIT:
                print("\n🛑 전방 장애물 감지 → 정지")
                state = "STOPPED"
                front_safe_count = 0
            elif rear >= RECOVERY_DISTANCE:
                rear_clear_count += 1
                if rear_clear_count >= RECOVERY_COUNT_REQUIRED:
                    print("\n✅ 후방 공간 확보 → 웨이포인트 주행 상태로 전환")
                    state = "WAYPOINT"
            else:
                rear_clear_count = 0

        elif state == "STOPPED":
            print("\n🚧 전방 충돌 위험으로 대기 중...")
            if front >= FRONT_COLLISION_LIMIT + SAFE_FRONT_MARGIN:
                front_safe_count += 1
                if front_safe_count >= FRONT_SAFE_COUNT_REQUIRED:
                    print("✅ 전방 안전 거리 확보 → 전진 재개")
                    state = "FORWARD"
                    rear_clear_count = 0  # 다시 전진 시 후방 확보 조건 재시작
            else:
                front_safe_count = 0

        elif state == "WAYPOINT":
            print("\n🛰️ 웨이포인트 따라 주행 중... (이후 통합 제어에서 처리 예정)")
            break

        time.sleep(0.1)

    serial_port.close()
    sys.exit()

if __name__ == "__main__":
    main()
