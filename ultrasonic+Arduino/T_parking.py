import serial
import time

# 하나의 보드만 연결됨
serial_port = serial.Serial('COM8', 9600, timeout=1)
time.sleep(2)

# 거리 기준 (cm)
STOP_THRESHOLD = 50
RECOVERY_DISTANCE = 100
STOP_COUNT_REQUIRED = 5  # 몇 번 이상 감지되어야 정지할지 설정

# 상태 정의
state = "REVERSING"
steer_reset_start_time = None
below_threshold_count = 0

def get_distance():
    line = serial_port.readline().decode('utf-8').strip()
    if line:
        try:
            _, dist = line.split(",")
            return float(dist)
        except ValueError:
            print("거리 파싱 실패:", line)
    return None

def main():
    global state, steer_reset_start_time, below_threshold_count

    while True:
        distance = get_distance()
        if distance is None:
            continue

        print(f"\n📏 현재 거리: {distance:.1f} cm | 상태: {state}")

        if state == "REVERSING":
            print("⏪ 차량 후진 중...")

            if distance <= STOP_THRESHOLD:
                below_threshold_count += 1
                print(f"⚠️ 감지 카운트: {below_threshold_count}")
            else:
                below_threshold_count = 0  # 조건 벗어나면 리셋

            if below_threshold_count >= STOP_COUNT_REQUIRED:
                print("🛑 차량 정지 (50cm 이하 거리 5회 감지)")
                state = "STEER_RESET"
                steer_reset_start_time = time.time()
                below_threshold_count = 0  # 초기화

        elif state == "STEER_RESET":
            print("↔️ 조향각 리셋 중...")
            elapsed = time.time() - steer_reset_start_time
            if elapsed >= 1.0:
                print("✅ 조향 리셋 완료 → 전진 시작")
                state = "FORWARD"
                steer_reset_start_time = None

        elif state == "FORWARD":
            print("🚗 차량 전진 중...")
            if distance >= RECOVERY_DISTANCE:
                print("🔁 공간 확보됨 → 다시 후진 시작")
                state = "REVERSING"

        time.sleep(0.1)

if __name__ == "__main__":
    main()
