import cv2
import json
import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32


CAMERA_INDEX = "/dev/v4l/by-id/usb-XZC-260109-A_Streaming_Webcam_Audio_01.00.00-video-index0"

MIN_AREA = 2500
KP = 0.002
KD = 0.001

PIXEL_DEADBAND = 15
MAX_OUTPUT = 0.35
REQUIRED_CENTERED_FRAMES = 3

OFFSET_RATIO_YELLOW_RIGHT = 0.2
OFFSET_RATIO_YELLOW_LEFT = -0.3


class PlantAlignmentNode(Node):

    def __init__(self):
        super().__init__('plant_alignment_node')

        self.positioning_enabled = False

        self.cap = None

        self.prev_error = 0.0
        self.prev_time = time.monotonic()

        self.centered_counter = 0

        (
            self.yellow_lower,
            self.yellow_upper,
            self.green_lower,
            self.green_upper
        ) = self.load_hsv_ranges()

        self.positioning_pub = self.create_publisher(
            Bool,
            '/plant_position_reached_camera',
            10
        )

        self.pid_pub = self.create_publisher(
            Float32,
            '/plant_pid_output_camera',
            10
        )

        self.detected_sub= self.create_subscription(
            Bool,
            '/plant_detected_camera',
            self.detected_callback,
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.process_frame
        )

    def load_hsv_ranges(
        self,
        path='/home/corn/ros2_mr26/src/calibration_image/colorrange.json'
    ):
        with open(path, 'r') as f:
            data = json.load(f)

        return (
            np.array(data['yellow_lower'], dtype=np.uint8),
            np.array(data['yellow_upper'], dtype=np.uint8),
            np.array(data['green_lower'], dtype=np.uint8),
            np.array(data['green_upper'], dtype=np.uint8),
        )

    def detected_callback(self, msg):

        if msg.data and not self.positioning_enabled:
            self.start_camera()

        elif not msg.data and self.positioning_enabled:
            self.stop_camera()

        self.positioning_enabled = msg.data

    def start_camera(self):

        self.get_logger().info("Starting camera")

        self.prev_time = time.monotonic()

        self.cap = cv2.VideoCapture(
            CAMERA_INDEX,
            cv2.CAP_V4L2
        )

        self.cap.set(cv2.CAP_PROP_FPS, 5)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000)

        time.sleep(2)

        for _ in range(5):
            self.cap.read()

        self.centered_counter = 0
        self.prev_error = 0.0

    def stop_camera(self):



        if self.cap is not None:
            self.cap.release()
            self.cap = None
            self.get_logger().info("Stopping camera")
            self.pid_pub.publish(Float32(data=0.0))

    def compute_pd_output(self, error):

        if abs(error) < PIXEL_DEADBAND:
            return 0.0

        now = time.monotonic()

        dt = now - self.prev_time

        if dt <= 0:
            dt = 0.001

        derivative = (
            error - self.prev_error
        ) / dt

        output = (
            KP * error +
            KD * derivative
        )

        self.prev_error = error
        self.prev_time = now

        return max(
            -MAX_OUTPUT,
            min(MAX_OUTPUT, output)
        )

    def process_frame(self):

        if not self.positioning_enabled:
            self.get_logger().warn(
                f"positioning_enabled={self.positioning_enabled}"
            )
            return

        if self.cap is None:
            return
        
        time.sleep(0.2)

        for _ in range(3):
            self.cap.read()

        ret, image = self.cap.read()

        if not ret:
            return

        frame_center_x = image.shape[1] // 2

        hsv = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV
        )

        yellow_mask = cv2.inRange(
            hsv,
            self.yellow_lower,
            self.yellow_upper
        )

        green_mask = cv2.inRange(
            hsv,
            self.green_lower,
            self.green_upper
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

        yellow = [
            c for c in yellow_contours
            if cv2.contourArea(c) > MIN_AREA
        ]

        green = [
            c for c in green_contours
            if cv2.contourArea(c) > MIN_AREA
        ]

        if not yellow:
            self.get_logger().warn("NO YELLOW CONTOUR")

        if not green:
            self.get_logger().warn("NO GREEN CONTOUR")

        self.get_logger().warn(
            f"yellow_contours={len(yellow)} "
            f"green_contours={len(green)}"
        )
        
        if not yellow or not green:
            self.pid_pub.publish(
                Float32(data=0.0)
            )

            self.positioning_pub.publish(
                Bool(data=False)
            )

            return

        yellow_contour = max(
            yellow,
            key=cv2.contourArea
        )

        green_contour = max(
            green,
            key=cv2.contourArea
        )

        x, y, w, h = cv2.boundingRect(
            yellow_contour
        )

        yellow_center_x = x + w // 2

        x, y, w, h = cv2.boundingRect(
            green_contour
        )

        green_center_x = x + w // 2

        distance = abs(
            yellow_center_x -
            green_center_x
        )

        if yellow_center_x > green_center_x:
            target_x = int(
                green_center_x +
                distance * OFFSET_RATIO_YELLOW_RIGHT
            )
        else:
            target_x = int(
                green_center_x +
                distance * OFFSET_RATIO_YELLOW_LEFT
            )

        pixel_error = (
            frame_center_x -
            target_x
        )

        pid_output = self.compute_pd_output(
            pixel_error
        )

        self.get_logger().warn(
            f"Y={yellow_center_x} "
            f"G={green_center_x} "
            f"T={target_x} "
            f"E={pixel_error} "
            f"PID={pid_output}"
        )

        self.pid_pub.publish(
            Float32(data=float(pid_output))
        )

        if abs(pixel_error) < PIXEL_DEADBAND:
            self.centered_counter += 1
        else:
            self.centered_counter = 0

        positionned = (
            self.centered_counter >=
            REQUIRED_CENTERED_FRAMES
        )

        self.positioning_pub.publish(
            Bool(data=positionned)
        )

    def destroy_node(self):

        if self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PlantAlignmentNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()