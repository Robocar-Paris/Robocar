from os import pipe
import cv2
import depthai as dai

pipeline = dai.Pipeline()

camera_color = pipeline.create(dai.node.ColorCamera)

camera_output = pipeline.create(dai.node.XLinkOut)
camera_output.setStreamName("rgb")

camera_color.setPreviewSize(300, 300)
camera_color.setInterleaved(False)
camera_color.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

camera_color.preview.link(camera_output.input)

i = 0

with dai.Device(pipeline) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        i += 1
        input_rgb = qRgb.get()
        cv2.imwrite(f"../mask_generator/datasets/CAMERA/camera_image_{i}.png", input_rgb.getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break