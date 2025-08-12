from ultralytics import YOLO
import depthai as dai
import cv2
import numpy as np
import time
import serial  # 시리얼 활성화

# 아두이노 시리얼 설정
SERIAL_PORT, SERIAL_BAUD = 'COM4', 115200
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

# 모델 로드
coco_model = YOLO('yolov8m.pt')
my_model   = YOLO(r"C:\\Users\\원수민\\HIGlobalMobility3Team12\\Camera(Traffic)+Arduino\\weights\\best.pt")
COCO_ID, MY_ID = 9, 0

# 신호등 색상 판별 함수
def get_traffic_light_color(roi):
    if roi.size == 0:
        return 'unknown', (255,255,255)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    masks = [
        cv2.inRange(hsv, (0,70,50), (15,255,255)) + cv2.inRange(hsv, (170,70,50), (180,255,255)), # 빨강
        cv2.inRange(hsv, (20,100,100), (35,255,255)), # 노랑
        cv2.inRange(hsv, (40,50,50), (90,255,255))    # 초록
    ]
    counts = [cv2.countNonZero(m) for m in masks]
    idx = np.argmax(counts)
    status_list = ['traffic_red','traffic_yellow','traffic_green']
    color_list  = [(0,0,255),(0,255,255),(0,255,0)]
    return (status_list[idx], color_list[idx]) if max(counts) > 5 else ('unknown',(255,255,255))

# DepthAI 파이프라인 설정
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setFps(30)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam_rgb.setInterleaved(False)
cam_rgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.OFF)
cam_rgb.initialControl.setManualFocus(130)

xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName('rgb')
cam_rgb.preview.link(xout_rgb.input)

# 스테레오 카메라 및 깊이 설정
mono_left  = pipeline.create(dai.node.MonoCamera)
mono_right = pipeline.create(dai.node.MonoCamera)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

stereo = pipeline.create(dai.node.StereoDepth)
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

xout_depth = pipeline.create(dai.node.XLinkOut)
xout_depth.setStreamName('depth')
stereo.depth.link(xout_depth.input)

# 장치 실행
with dai.Device(pipeline) as device:
    q_rgb   = device.getOutputQueue('rgb',   maxSize=4, blocking=False)
    q_depth = device.getOutputQueue('depth', maxSize=4, blocking=False)
    print('실행 중... 종료: q')

    frame_count, interval = 0, 5
    prev_boxes = []

    while True:
        start = time.time()
        frame       = q_rgb.get().getCvFrame()
        depth_frame = q_depth.get().getFrame()
        h, w = frame.shape[:2]

        boxes = []
        frame_count += 1

        if frame_count % interval == 0:
            signal_to_send = None  # 아두이노로 보낼 값

            for model, tid, prefix in [(coco_model, COCO_ID, ''),(my_model, MY_ID, 'MY_')]:
                try:
                    res = model.predict(frame, verbose=False, device='cpu')[0]
                except Exception as e:
                    print(prefix+' 모델 오류:', e)
                    continue

                for b in res.boxes:
                    cls, conf = int(b.cls[0]), float(b.conf[0])
                    if cls != tid or conf < 0.6:
                        continue

                    x1,y1,x2,y2 = map(int, b.xyxy[0])
                    cx, cy = (x1+x2)//2, (y1+y2)//2
                    dx = int(cx*depth_frame.shape[1]/w)
                    dy = int(cy*depth_frame.shape[0]/h)
                    dist = depth_frame[dy,dx].item()/1000
                    status,color = get_traffic_light_color(frame[y1:y2,x1:x2])

                    boxes.append((x1,y1,x2,y2,color,f"{prefix}{status} {conf:.2f} ({dist:.2f}m)"))

                    # 신호 전송 조건
                    if status in ['traffic_red','traffic_yellow'] and conf >= 0.6:
                        signal_to_send = '0'
                    elif status == 'traffic_green' and conf >= 0.6:
                        signal_to_send = '1'

                    if signal_to_send is not None:
                        ser.write(signal_to_send.encode())  # 아두이노로 송신
                        print(f"아두이노 전송: {signal_to_send}")
                        break
                if signal_to_send is not None:
                    break

            prev_boxes = boxes
        else:
            boxes = prev_boxes

        for x1,y1,x2,y2,col,label in boxes:
            cv2.rectangle(frame,(x1,y1),(x2,y2),col,2)
            cv2.putText(frame,label,(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX,0.7,col,2)

        cv2.imshow('OAK-D 신호등 인식', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        elapsed = time.time() - start
        fps = 1/elapsed if elapsed > 0 else 0
        print(f"시간:{elapsed:.3f}s, FPS:{fps:.1f}")

cv2.destroyAllWindows()
ser.close()
