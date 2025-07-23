from ultralytics import YOLO
import depthai as dai
import cv2
import numpy as np

# --- 모델 불러오기 ---
coco_model = YOLO('yolov8m.pt')
COCO_TRAFFIC_LIGHT_ID = 9
COCO_PERSON_ID = 0
COCO_CAR_IDS = [2, 3, 5, 7]  # car, motorcycle, bus, truck

my_model = YOLO(r"C:\Users\원수민\HIGlobalMobility3Team12\Camera(Traffic)+Arduino\weights\best.pt")
MY_TRAFFIC_LIGHT_ID = 0

status_to_color = {
    "traffic_red": (0, 0, 255),
    "traffic_yellow": (0, 255, 255),
    "traffic_green": (0, 255, 0),
    "unknown": (255, 255, 255)
}

def get_traffic_light_color(roi):
    """HSV 색상으로 신호등 색상 판단"""
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
    status = status_list[idx] if max(counts)>5 else "unknown"
    color = color_list[idx] if max(counts)>5 else (255,255,255)
    return status, color

pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam_rgb.setInterleaved(False)
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

mono_left = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

stereo = pipeline.create(dai.node.StereoDepth)
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)
xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    print("OAK-D Pro 신호등/사람/차량 인식 (COCO+내모델, HSV 이중색상판별)")

    while True:
        in_rgb = q_rgb.get()
        frame = in_rgb.getCvFrame()
        in_depth = q_depth.get()
        depth_frame = in_depth.getFrame()

        rgb_h, rgb_w = frame.shape[:2]
        depth_h, depth_w = depth_frame.shape[:2]

        # --- stop_signal 기본값 (움직임) ---
        stop_signal = False

        # === COCO 모델 ===
        coco_results = coco_model.predict(frame, verbose=False)[0]
        for box in coco_results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = x1 + (x2-x1)//2
            cy = y1 + (y2-y1)//2
            cx_depth = int(cx * depth_w / rgb_w)
            cy_depth = int(cy * depth_h / rgb_h)
            cx_depth = min(max(cx_depth, 0), depth_w-1)
            cy_depth = min(max(cy_depth, 0), depth_h-1)
            depth_value = depth_frame[cy_depth, cx_depth].item()

            # 신호등
            if cls == COCO_TRAFFIC_LIGHT_ID:
                roi = frame[y1:y2, x1:x2]
                status, color = get_traffic_light_color(roi)  # HSV 이중판별
                label = f"{status} {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

                # 신호등 색상 메시지 + STOP/GO 신호
                if status == "traffic_red":
                    print("COCO: 신호등 빨간색입니다.\nSTOP 신호 (빨간불) - 차가 멈춰야 합니다.")
                    stop_signal = True
                elif status == "traffic_yellow":
                    print("COCO: 신호등 노란색입니다.\nSTOP 신호 (노란불) - 차가 멈춰야 합니다.")
                    stop_signal = True
                elif status == "traffic_green":
                    print("COCO: 신호등 초록입니다.\nGO 신호 (초록불) - 차가 진행할 수 있습니다.")

            # 사람
            elif cls == COCO_PERSON_ID:
                color = (255, 200, 0)
                label = f"Person {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

            # 차량 (자동차, 오토바이, 버스, 트럭)
            elif cls in COCO_CAR_IDS:
                color = (255, 0, 0)  # 파랑 계열 (자동차)
                car_classes = ["Car", "Motorcycle", "Bus", "Truck"]
                class_str = car_classes[COCO_CAR_IDS.index(cls)]
                label = f"{class_str} {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

        # === 내 모델 (신호등만) ===
        my_results = my_model.predict(frame, verbose=False)[0]
        for box in my_results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = x1 + (x2-x1)//2
            cy = y1 + (y2-y1)//2
            cx_depth = int(cx * depth_w / rgb_w)
            cy_depth = int(cy * depth_h / rgb_h)
            cx_depth = min(max(cx_depth, 0), depth_w-1)
            cy_depth = min(max(cy_depth, 0), depth_h-1)
            depth_value = depth_frame[cy_depth, cx_depth].item()

            if cls == MY_TRAFFIC_LIGHT_ID:
                roi = frame[y1:y2, x1:x2]
                status, color = get_traffic_light_color(roi)  # 내 모델도 HSV 이중판별!
                label = f"MY_{status} {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1+25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

                # 신호등 색상 메시지 + STOP/GO 신호
                if status == "traffic_red":
                    print("MY: 신호등 빨간색입니다.\nSTOP 신호 (빨간불) - 차가 멈춰야 합니다.")
                elif status == "traffic_yellow":
                    print("MY: 신호등 노란색입니다.\nSTOP 신호 (노란불) - 차가 멈춰야 합니다.")
                elif status == "traffic_green":
                    print("MY: 신호등 초록입니다.\nGO 신호 (초록불) - 차가 진행할 수 있습니다.")

        cv2.imshow("OAK-D Pro 신호등/사람/차량 인식", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

cv2.destroyAllWindows()
