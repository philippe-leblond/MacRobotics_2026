import cv2
import numpy as np
import json
import time

CAMERA_INDEX = "/dev/v4l/by-id/usb-XZC-260109-A_Streaming_Webcam_Audio_01.00.00-video-index0"

MIN_AREA = 2500
FRAME_RATE = 15

KP = 0.003
KD = 0.001

PIXEL_DEADBAND = 15
MAX_OUTPUT = 0.25

REQUIRED_CENTERED_FRAMES = 3

OFFSET_RATIO_YELLOW_RIGHT = 0.2
OFFSET_RATIO_YELLOW_LEFT = -0.3

prev_error = 0.0
prev_time = time.monotonic()

centered_counter = 0
align = True


def load_hsv_ranges(path='/home/corn/ros2_mr26/src/calibration_image/colorrange.json'):

    with open(path, 'r') as f:
        data = json.load(f)

    return (
        np.array(data['yellow_lower'], dtype=np.uint8),
        np.array(data['yellow_upper'], dtype=np.uint8),
        np.array(data['green_lower'], dtype=np.uint8),
        np.array(data['green_upper'], dtype=np.uint8),
    )


(
    yellow_lower_range,
    yellow_upper_range,
    green_lower_range,
    green_upper_range
) = load_hsv_ranges()


def compute_pd_output(error):

    global prev_error
    global prev_time

    if abs(error) < PIXEL_DEADBAND:
        return 0.0

    now = time.monotonic()

    dt = now - prev_time

    if dt <= 0.0:
        dt = 0.001

    derivative = (error - prev_error) / dt

    output = (
        KP * error +
        KD * derivative
    )

    prev_error = error
    prev_time = now

    return max(
        -MAX_OUTPUT,
        min(MAX_OUTPUT, output)
    )


cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

if not cap.isOpened():
    raise RuntimeError("Failed to open camera")

# Match calibration settings
cap.set(cv2.CAP_PROP_FPS, 5)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Fixed white balance
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000)

# Allow camera to settle
time.sleep(2)

# Flush stale frames
for _ in range(5):
    cap.read()

while True:

    ret, image = cap.read()

    if not ret:
        continue

    frame_center_x = image.shape[1] // 2

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    yellow_mask = cv2.inRange(
        hsv,
        yellow_lower_range,
        yellow_upper_range
    )

    green_mask = cv2.inRange(
        hsv,
        green_lower_range,
        green_upper_range
    )

    yellow_contours, _ = cv2.findContours(
        yellow_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    green_contours, _ = cv2.findContours(
        green_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    yellow_center_x = None
    green_center_x = None

    large_yellow = [
        c for c in yellow_contours
        if cv2.contourArea(c) > MIN_AREA
    ]

    large_green = [
        c for c in green_contours
        if cv2.contourArea(c) > MIN_AREA
    ]

    if large_yellow:

        contour = max(
            large_yellow,
            key=cv2.contourArea
        )

        x, y, w, h = cv2.boundingRect(contour)

        yellow_center_x = x + w // 2

        cv2.rectangle(
            image,
            (x, y),
            (x + w, y + h),
            (0, 255, 255),
            2
        )

    if large_green:

        contour = max(
            large_green,
            key=cv2.contourArea
        )

        x, y, w, h = cv2.boundingRect(contour)

        green_center_x = x + w // 2

        cv2.rectangle(
            image,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )

    position_reached = False

    if (
        green_center_x is not None and
        yellow_center_x is not None
    ):

        # Start from center of green plant
        global_center_x = green_center_x

        distance = abs(yellow_center_x - green_center_x)



        if yellow_center_x > green_center_x:
            global_center_x = int(
                green_center_x + distance * OFFSET_RATIO_YELLOW_RIGHT
            )
        else:
            global_center_x = int(
                green_center_x + distance * OFFSET_RATIO_YELLOW_LEFT
            )

        pixel_error = (
            frame_center_x -
            global_center_x
        )

        pid_output = compute_pd_output(
            pixel_error
        )

        if abs(pixel_error) < PIXEL_DEADBAND:
            centered_counter += 1
        else:
            centered_counter = 0

        position_reached = (
            centered_counter >=
            REQUIRED_CENTERED_FRAMES
        )

        if position_reached:
            align = False

        cv2.line(
            image,
            (frame_center_x, 0),
            (frame_center_x, image.shape[0]),
            (255, 0, 0),
            2
        )

        cv2.circle(
            image,
            (global_center_x,
             image.shape[0] // 2),
            10,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            image,
            f"Error: {pixel_error:.1f}px",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            image,
            f"PD Output: {pid_output:.3f}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            image,
            f"Reached: {position_reached}",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            f"Align: {align}",
            (20, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

    cv2.imshow(
        "Corn Alignment Test",
        image
    )

    key = cv2.waitKey(1)

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()