import depthai as dai
import cv2
import numpy as np

# Build a simple color-camera pipeline (same idea as test_camera.py)
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(1280, 720)          # small preview = smooth tuning
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.preview.link(xout.input)

# Create a window with 6 sliders for the HSV range
cv2.namedWindow("Tuner")
cv2.createTrackbar("H min", "Tuner", 30,  179, lambda x: None)
cv2.createTrackbar("H max", "Tuner", 90,  179, lambda x: None)
cv2.createTrackbar("S min", "Tuner", 60,  255, lambda x: None)
cv2.createTrackbar("S max", "Tuner", 255, 255, lambda x: None)
cv2.createTrackbar("V min", "Tuner", 60,  255, lambda x: None)
cv2.createTrackbar("V max", "Tuner", 255, 255, lambda x: None)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("video", maxSize=4, blocking=False)
    print("Drag sliders until ONLY the probe is white in the mask. Press 'q' to quit.")
    while True:
        frame = q.get().getCvFrame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Read current slider positions
        hmin = cv2.getTrackbarPos("H min", "Tuner")
        hmax = cv2.getTrackbarPos("H max", "Tuner")
        smin = cv2.getTrackbarPos("S min", "Tuner")
        smax = cv2.getTrackbarPos("S max", "Tuner")
        vmin = cv2.getTrackbarPos("V min", "Tuner")
        vmax = cv2.getTrackbarPos("V max", "Tuner")

        lower = np.array([hmin, smin, vmin])
        upper = np.array([hmax, smax, vmax])
        mask = cv2.inRange(hsv, lower, upper)

        # Show original and mask side by side-ish
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("Original", frame)
        cv2.imshow("Mask (white = detected)", mask)
        cv2.imshow("Result", result)

        if cv2.waitKey(1) == ord('q'):
            print(f"\nYour HSV range:")
            print(f"  lower = [{hmin}, {smin}, {vmin}]")
            print(f"  upper = [{hmax}, {smax}, {vmax}]")
            break

cv2.destroyAllWindows()