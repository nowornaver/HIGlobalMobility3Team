from ultralytics import YOLO
import depthai as dai
import cv2
import numpy as np
# import serial
import time

# 시리얼 포트 설정
# SERIAL_PORT = 'COM4'
# SERIAL_BAUD = 115200
# ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

# 모델 로드
coco_model = YOLO('yolov8n.pt')
COCO_TRAFFIC_LIGHT_ID = 9
my_model = YOLO(r"C:\Users\원수민\HIGlobalMobility3Team12\Camera(Traffic)+Arduino\weights\best.pt")
MY_TRAFFIC_LIGHT_ID = 0

def get_traffic_light_color(roi):
    if roi.size == 0:
        return 'unknown', (255,255,255)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) + \
               cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
    mask_yellow = cv2.inRange(hsv, (20, 100, 100), (35, 255, 255))
    mask_green = cv2.inRange(hsv, (40, 50, 50), (90, 255, 255))
    counts = [cv2.countNonZero(mask_red), cv2.countNonZero(mask_yellow), cv2.countNonZero(mask_green)]
    idx = np.argmax(counts)
    status_list = ["traffic_red", "traffic_yellow", "traffic_green"]
    color_list = [(0,0,255), (0,255,255), (0,255,0)]
    status = status_list[idx] if max(counts) > 5 else "unknown"
    color = color_list[idx] if max(counts) > 5 else (255,255,255)
    return status, color

# 파이프라인 생성
pipeline = dai.Pipeline()

# 컬러 카메라 설정
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setFps(30)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam_rgb.setInterleaved(False)
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

# 스테레오 카메라 설정
mono_left = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_left.setFps(30)
mono_right.setFps(30)
mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

# 깊이 노드
stereo = pipeline.create(dai.node.StereoDepth)
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)
xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# 변수 초기화
frame_count = 0
yolo_interval = 5
prev_boxes = []
prev_state = "FORWARD"
last_sent_state = ""

# 장치 실행
with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    print("실행 중... 종료하려면 q 키")

    while True:
        start = time.time()

        in_rgb = q_rgb.get()
        frame = in_rgb.getCvFrame()
        in_depth = q_depth.get()
        depth_frame = in_depth.getFrame()

        rgb_h, rgb_w = frame.shape[:2]
        depth_h, depth_w = depth_frame.shape[:2]

        stop_signal = False
        boxes = []
        frame_count += 1

        if frame_count % yolo_interval == 0:
            try:
                coco_results = coco_model.predict(frame, verbose=False, device="cpu")[0]
            except Exception as e:
                print("COCO 모델 예측 오류:", e)
                continue

            found = False
            for box in coco_results.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = x1 + (x2 - x1) // 2
                cy = y1 + (y2 - y1) // 2
                cx_depth = int(cx * depth_w / rgb_w)
                cy_depth = int(cy * depth_h / rgb_h)
                cx_depth = np.clip(cx_depth, 0, depth_w - 1)
                cy_depth = np.clip(cy_depth, 0, depth_h - 1)
                depth_value = depth_frame[cy_depth, cx_depth].item()
                if cls == COCO_TRAFFIC_LIGHT_ID:
                    roi = frame[y1:y2, x1:x2]
                    status, color = get_traffic_light_color(roi)
                    label = f"{status} {conf:.2f} ({depth_value/1000:.2f}m)"
                    boxes.append((x1, y1, x2, y2, color, label))
                    if status in ["traffic_red", "traffic_yellow"] and conf >= 0.6:
                        stop_signal = True
                        prev_state = "STEER_RESET"
                        found = True
                        break
                    elif status == "traffic_green" and conf >= 0.6:
                        stop_signal = False
                        prev_state = "FORWARD"
                        found = True
                        break

            if not found:
                try:
                    my_results = my_model.predict(frame, verbose=False, device="cpu")[0]
                except Exception as e:
                    print("MY 모델 예측 오류:", e)
                    continue

                for box in my_results.boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = x1 + (x2 - x1) // 2
                    cy = y1 + (y2 - y1) // 2
                    cx_depth = int(cx * depth_w / rgb_w)
                    cy_depth = int(cy * depth_h / rgb_h)
                    cx_depth = np.clip(cx_depth, 0, depth_w - 1)
                    cy_depth = np.clip(cy_depth, 0, depth_h - 1)
                    depth_value = depth_frame[cy_depth, cx_depth].item()
                    if cls == MY_TRAFFIC_LIGHT_ID:
                        roi = frame[y1:y2, x1:x2]
                        status, color = get_traffic_light_color(roi)
                        label = f"MY_{status} {conf:.2f} ({depth_value/1000:.2f}m)"
                        boxes.append((x1, y1, x2, y2, color, label))
                        if status in ["traffic_red", "traffic_yellow"] and conf >= 0.6:
                            stop_signal = True
                            prev_state = "STEER_RESET"
                            break
                        elif status == "traffic_green" and conf >= 0.6:
                            stop_signal = False
                            prev_state = "FORWARD"
                            break

            prev_boxes = boxes
        else:
            boxes = prev_boxes

        # 시각화
        for x1, y1, x2, y2, color, label in boxes:
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 상태가 바뀌었을 때만 아두이노로 전송
        # if prev_state != last_sent_state:
        #     try:
        #         ser.write(f"{prev_state},{0}\n".encode())
        #         last_sent_state = prev_state
        #     except Exception as e:
        #         print("Serial Write Error:", e)

        # 프레임 디스플레이
        cv2.imshow("OAK-D Pro 신호등 인식", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # 프레임 처리 시간 출력
        elapsed = time.time() - start
        print(f"프레임 시간: {elapsed:.3f}s, FPS: {1/elapsed:.1f}")

cv2.destroyAllWindows()
