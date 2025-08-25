##아두이노로 보내는 yaw값 보내는 코드 추가 바람.
import depthai as dai
import time, math
import serial

# 1) 파이프라인 구성
pipeline = dai.Pipeline()
imu = pipeline.create(dai.node.IMU)

# 자이로/가속도 활성화 (100Hz). yaw rate만 쓰려면 자이로만 켜도 됨.
imu.enableIMUSensor([dai.IMUSensor.GYROSCOPE_RAW, dai.IMUSensor.ACCELEROMETER_RAW], 100)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("imu")
imu.out.link(xout.input)

# 2) 실행 및 읽기
yaw_deg = 0.0
prev_t = None

def integrate_yaw(yaw_deg, gz_rad_s, dt):
    # gz: rad/s, dt: s -> 적분해서 deg로 누적
    return yaw_deg + (gz_rad_s * dt * 180.0 / math.pi)

yaw_zero = None  # 처음 한 번만 현재 yaw를 기준으로 저장
def _wrap_signed_deg(a):
    return (a + 180.0) % 360.0 - 180.0

dy_1s = 0.0
t_1s = None

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("imu", maxSize=50, blocking=False)
    print("Reading OAK-D Pro IMU... (Ctrl+C to stop)")
    try:
        while True:
            pkt = q.get()  # IMUData
            now = time.monotonic()
            if t_1s is None:
                t_1s = now

            # 패킷에 여러 샘플이 있을 수 있음
            for p in pkt.packets:
                gz = p.gyroscope.z  # rad/s, 보통 Z축이 yaw 축
                # yaw 변화량(각속도)은 그대로 gz (rad/s) -> deg/s로 변환해서 보기 좋게 출력
                yaw_rate_dps = gz * 180.0 / math.pi

                # yaw 각도 적분
                if prev_t is None:
                    prev_t = now
                dt = now - prev_t
                prev_t = now

                yaw_deg = integrate_yaw(yaw_deg, gz, dt)


                if yaw_zero is None:
                    yaw_zero = yaw_deg  # 시작 시점의 yaw를 기준으로 고정
                yaw_rel = _wrap_signed_deg(yaw_deg - yaw_zero)
                delta_yaw_deg = yaw_rate_dps * dt
                dy_1s += delta_yaw_deg
                # print(f"yaw: {yaw_rel:8.2f} °")
                print(f"yaw_rate: {yaw_rate_dps:7.2f} °/s | yaw: {yaw_rel:8.2f} °")
                
                if (now - t_1s) >= 1.0:
                    avg_rate_1s = dy_1s / (now - t_1s)  # ≈ °/s
                    print(f"Δyaw@1s: {dy_1s:6.2f} ° | avg_rate@1s: {avg_rate_1s:6.2f} °/s")
                    dy_1s = 0.0
                    t_1s = now
                # if (now - t_1s) >= 1.0:
                #     avg_rate_1s = dy_1s / (now - t_1s)  # ≈ °/s
                #     print(f"Δyaw@1s: {dy_1s:6.2f} ° | avg_rate@1s: {avg_rate_1s:6.2f} °/s")
                #     dy_1s = 0.0
                #     t_1s = now

    except KeyboardInterrupt:
        print("exit")
