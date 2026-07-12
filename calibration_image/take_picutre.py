import cv2
import time
import os

CAMERA_INDEX = "/dev/v4l/by-id/usb-XZC-260109-A_Streaming_Webcam_Audio_01.00.00-video-index0"
OUTPUT_PATH = "/home/corn/ros2_mr26/src/calibration_image/corn.jpg"

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FPS, 5)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 1000)

if not cap.isOpened():
    print("ERROR: Could not open camera")
    exit()

# Let camera stabilize
time.sleep(2)

# Flush old frames
for _ in range(5):
    cap.read()

ret, frame = cap.read()

if ret:
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    cv2.imwrite(OUTPUT_PATH, frame)
    print(f"Saved image to: {OUTPUT_PATH}")
else:
    print("ERROR: Failed to capture image")

cap.release()