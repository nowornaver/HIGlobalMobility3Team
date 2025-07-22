from ultralytics import YOLO
import depthai as dai
import cv2
import numpy as np

# --- 모델 불러오기 ---
coco_model = YOLO('yolov8m.pt')
COCO_TRAFFIC_LIGHT_ID = 9
COCO_PERSON_ID = 0

my_model = YOLO(r"C:\Users\원수민\Desktop\yolov8\runs\detect\train\weights\best.pt")
MY_TRAFFIC_LIGHT_ID = 0

status_to_color = {
    "traffic_red": (0, 0, 255),
    "traffic_yellow": (0, 255, 255),
    "traffic_green": (0, 255, 0),
    "unknown": (255, 255, 255)
}

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
    print("OAK-D Pro 신호등/사람 인식")

    while True:
        in_rgb = q_rgb.get()
        frame = in_rgb.getCvFrame()
        in_depth = q_depth.get()
        depth_frame = in_depth.getFrame()

        rgb_h, rgb_w = frame.shape[:2]
        depth_h, depth_w = depth_frame.shape[:2]

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

            if cls == COCO_TRAFFIC_LIGHT_ID:
                roi = frame[y1:y2, x1:x2]
                if roi.size > 0:
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    mask_red = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) + \
                               cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
                    mask_yellow = cv2.inRange(hsv, (20, 100, 100), (35, 255, 255))
                    mask_green = cv2.inRange(hsv, (40, 50, 50), (90, 255, 255))
                    counts = [cv2.countNonZero(mask_red), cv2.countNonZero(mask_yellow), cv2.countNonZero(mask_green)]
                    idx = np.argmax(counts)
                    status = ["traffic_red", "traffic_yellow", "traffic_green"][idx]
                else:
                    status = "unknown"
                color = status_to_color[status]
                label = f"{status} {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

                # --- 신호등 색상에 따른 메시지 출력 (CMD) ---
                if status == "traffic_red":
                    print("신호등 빨간색입니다.")
                elif status == "traffic_yellow":
                    print("신호등 노란색입니다.")
                elif status == "traffic_green":
                    print("신호등 초록입니다.")

            elif cls == COCO_PERSON_ID:
                color = (255, 200, 0)
                label = f"Person {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

        # === 내 모델 ===
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
                if roi.size > 0:
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    mask_red = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) + \
                               cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
                    mask_yellow = cv2.inRange(hsv, (20, 100, 100), (35, 255, 255))
                    mask_green = cv2.inRange(hsv, (40, 50, 50), (90, 255, 255))
                    counts = [cv2.countNonZero(mask_red), cv2.countNonZero(mask_yellow), cv2.countNonZero(mask_green)]
                    idx = np.argmax(counts)
                    status = ["traffic_red", "traffic_yellow", "traffic_green"][idx]
                else:
                    status = "unknown"
                color = status_to_color[status]
                label = f"{status} {conf:.2f} ({depth_value/1000:.2f}m)"
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1+25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

                # --- 신호등 색상에 따른 메시지 출력 (CMD) ---
                if status == "traffic_red":
                    print("신호등 빨간색입니다.")
                elif status == "traffic_yellow":
                    print("신호등 노란색입니다.")
                elif status == "traffic_green":
                    print("신호등 초록입니다.")

        cv2.imshow("OAK-D Pro 신호등/사람 인식", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

cv2.destroyAllWindows()
