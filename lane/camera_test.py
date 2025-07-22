import depthai as dai
import cv2

pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(1280, 720)
cam_rgb.setInterleaved(False)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

controlIn = pipeline.create(dai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(cam_rgb.inputControl)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("rgb")
cam_rgb.preview.link(xout.input)

with dai.Device(pipeline) as device:
    # 수동 포커스 값 0~255 (100~120정도 추천, 직접 맞춰야함)
    ctrl = dai.CameraControl()
    ctrl.setManualFocus(200)  # 숫자 조절해가며 테스트 (보통 90~150 사이)
    controlQ = device.getInputQueue('control')
    controlQ.send(ctrl)

    q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    while True:
        in_rgb = q.get()
        frame = in_rgb.getCvFrame()
        cv2.imshow("OAK-D Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
