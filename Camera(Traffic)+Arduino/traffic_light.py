from ultralytics import YOLO
import depthai as dai
import cv2
import numpy as np
import serial

# 시리얼(아두이노) 설정
SERIAL_PORT = 'COM3'  # 실제 연결된 포트로 바꿔야 함!
SERIAL_BAUD = 9600
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

# --- 모델 불러오기 ---
coco_model = YOLO('yolov8m.pt')
COCO_TRAFFIC_LIGHT_ID = 9
COCO_PERSON_ID = 0
COCO_CAR_IDS = [2, 3, 5, 7]
state = ""
my_model = YOLO(r"C:\Users\원수민\HIGlobalMobility3Team12\Camera(Traffic)+Arduino\weights\best.pt")
MY_TRAFFIC_LIGHT_ID = 0

status_to_color = {
    "traffic_red": (0, 0, 255),
    "traffic_yellow": (0, 255, 255),
    "traffic_green": (0, 255, 0),
    "unknown": (255, 255, 255)
}

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
    print("OAK-D Pro 신호등/사람/차량 인식")

    while True:
        in_rgb = q_rgb.get()
        frame = in_rgb.getCvFrame()
        in_depth = q_depth.get()
        depth_frame = in_depth.getFrame()

        rgb_h, rgb_w = frame.shape[:2]
        depth_h, depth_w = depth_frame.shape[:2]

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
                status, color = get_traffic_light_color(roi)
                label = f"{status} {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

                if status in ["traffic_red", "traffic_yellow"] and conf >= 0.6:
                    print(f"COCO: {status} conf={conf:.2f} - STOP")
                    stop_signal = True
                    break

            elif cls == COCO_PERSON_ID:
                color = (255, 200, 0)
                label = f"Person {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)
            elif cls in COCO_CAR_IDS:
                color = (255, 0, 0)
                car_classes = ["Car", "Motorcycle", "Bus", "Truck"]
                class_str = car_classes[COCO_CAR_IDS.index(cls)]
                label = f"{class_str} {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

        if not stop_signal:
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
                    status, color = get_traffic_light_color(roi)
                    label = f"MY_{status} {conf:.2f} ({depth_value/1000:.2f}m)"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, label, (x1, y1+25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    cv2.circle(frame, (cx, cy), 5, color, -1)

                    if status in ["traffic_red", "traffic_yellow"] and conf >= 0.6:
                        print(f"MY: {status} conf={conf:.2f} - STOP")
                        stop_signal = True
                        break
        # --- 실제 아두이노 연동 예시 ---
        if stop_signal:
            state = "STEER_RESET"
            ser.write(b'STEER_RESET\n')
            ser.write()

        else:
            state = "FORWARD"

    cmd_str = f"{state},{0}\n"
    ser.write(cmd_str.encode('utf-8'))
        cv2.imshow("OAK-D Pro 신호등/사람/차량 인식", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

cv2.destroyAllWindows()