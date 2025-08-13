import depthai as dai
# import serial
# import time

# =========================
# 1) 아두이노 시리얼 설정
# =========================
# SERIAL_PORT = 'COM4'      # 환경에 맞게 변경
# SERIAL_BAUD = 9600
# ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
# time.sleep(2)  # 아두이노 자동 리셋 대기

# =========================
# 2) DepthAI 파이프라인 구성
# =========================
pipeline = dai.Pipeline()

# -- 좌/우 모노 카메라 (스테레오 깊이 입력용)
mono_left = pipeline.createMonoCamera()
mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

mono_right = pipeline.createMonoCamera()
mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

# -- 스테레오 깊이 모듈
stereo = pipeline.createStereoDepth()
stereo.initialConfig.setConfidenceThreshold(200)  # setConfidenceThreshold() (deprecated) 대체
# 품질 향상 옵션(원하면 사용)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
stereo.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_7x7)

mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

# -- SpatialLocationCalculator (ROI 거리 계산)
spatial_calc = pipeline.createSpatialLocationCalculator()
spatial_calc.inputConfig.setWaitForMessage(False)  # setWaitForConfigInput(False) (deprecated) 대체

cfg = dai.SpatialLocationCalculatorConfigData()
cfg.depthThresholds.lowerThreshold = 100     # 0.1 m
cfg.depthThresholds.upperThreshold = 15000    # 4.0 m
# 중앙 40~60% 정규좌표 ROI
cfg.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))
spatial_calc.initialConfig.addROI(cfg)

# 깊이 → SLC 입력
stereo.depth.link(spatial_calc.inputDepth)

# -- 호스트로 SLC 결과 전송
xout_spatial = pipeline.createXLinkOut()
xout_spatial.setStreamName("spatialData")
spatial_calc.out.link(xout_spatial.input)

# =========================
# 3) 실행 루프
# =========================
with dai.Device(pipeline) as device:
    q_spatial = device.getOutputQueue(name="spatialData", maxSize=4, blocking=True)

    last_signal = None  # 같은 값 반복 전송 방지

    print(">>> 시작: 0.1m ~ 4.0m → '0'(정지), 그 외 → '1'(전진)")

    try:
        while True:
            packet = q_spatial.get()  # 새 데이터까지 대기
            locs = packet.getSpatialLocations()
            if not locs:
                continue

            # ROI가 1개이므로 0번 사용
            z_mm = locs[0].spatialCoordinates.z
            dist_m = z_mm / 1000.0 if z_mm is not None else float('inf')

            # 무효(0 또는 음수) 값 예외 처리
            if z_mm is None or z_mm <= 0:
                # 측정 불가 → 전진(=1)
                signal = '1'
            else:
                # 0.1m ~ 4.0m → 0, 그 외 → 1
                signal = '0' if (0.1 <= dist_m <= 4.0) else '1'

            # 값이 바뀔 때만 전송
            if signal != last_signal:
                # 3ser.write(signal.encode())  # 한 글자 전송(개행 없음)
                print(f"[TX] {signal}  |  Distance: {dist_m:.2f} m")
                last_signal = signal
            else:
                print(f"Distance: {dist_m:.2f} m (hold {signal})")

    except KeyboardInterrupt:
        pass
    finally:
        # ser.close()
        print("종료 및 시리얼 닫힘.") 
