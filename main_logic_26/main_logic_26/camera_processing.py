import cv2
import numpy as np
import json
import rclpy
import os
import time
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Int32MultiArray
from ament_index_python.packages import get_package_share_directory


#CAMERA_INDEX = "/dev/video0"
CAMERA_INDEX = "/dev/v4l/by-id/usb-XZC-260109-A_Streaming_Webcam_Audio_01.00.00-video-index0"
FRAME_RATE = 5

MIN_AREA = 2500
HEIGHT_THRESHOLD = 125


# -----------------------------
# Calibration loader
# -----------------------------

def load_hsv_ranges():
    pkg_path = get_package_share_directory('main_logic_26')
    file_path = os.path.join(pkg_path, 'config', 'colorrange.json')
    
    #print("DEBUG path:", file_path)  

    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
            #print("DEBUG: JSON loaded successfully")

        yl = data.get('yellow_lower')
        yu = data.get('yellow_upper')
        gl = data.get('green_lower')
        gu = data.get('green_upper')

        if not all(isinstance(x, list) for x in (yl, yu, gl, gu)):
            return None

        return (
            np.array(yl, dtype=np.uint8),
            np.array(yu, dtype=np.uint8),
            np.array(gl, dtype=np.uint8),
            np.array(gu, dtype=np.uint8),
        )
    except Exception:
        return None


# -----------------------------
# Node Class
# -----------------------------
class CameraProcessingNode(Node):

    def __init__(self):
        super().__init__('corn_vision')

        self.debug_dir = "/home/corn/ros2_mr26/src/main_logic_26/main_logic_26/debug_pictures"

        os.makedirs(self.debug_dir, exist_ok=True)

        self.capture_count = 0


        # Load HSV ranges
        
        package_path = get_package_share_directory('main_logic_26')
        json_path = os.path.join(package_path, 'colorrange.json')

        loaded = load_hsv_ranges()

        if loaded is None:
            self.get_logger().error("Failed to load colorrange.json")
            raise RuntimeError("HSV config missing")

        self.yellow_lower, self.yellow_upper, self.green_lower, self.green_upper = loaded

        self.get_logger().info("Camera will be opened on each request")

        # Store results for one capture cycle
        self.plant_results = [0,0,0]


        # Publisher
        self.camera_choice_pub = self.create_publisher(Int32, 'camera_choice', 10)
        self.led_control_pub = self.create_publisher(Int32, 'led_control', 10)
        self.display_pub = self.create_publisher(Int32MultiArray, 'display_control', 10)


        # Subscriber (matches your state machine)
        self.create_subscription(
            Bool,
            '/camera_request',
            self.capture_trigger_callback,
            10
        )
    
    # -----------------------------    
    # One capture    
    # -----------------------------    
    def capture_one_frame(self):
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

        try:
            cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000)

            if not cap.isOpened():
                self.get_logger().warn("Failed to open camera")
                return None

            time.sleep(0.5)

            for _ in range(3):
                cap.read()

            ret, frame = cap.read()

            if not ret:
                self.get_logger().warn("Frame capture failed")
                return None

            return frame

        finally:
            cap.release()
    # -----------------------------
    # Publishers
    # -----------------------------
    def publish_camera_choice(self, choice):
        msg = Int32()
        msg.data = choice
        self.camera_choice_pub.publish(msg)
        self.get_logger().info(f"Published camera choice: {choice}")
    
    def publish_led_control(self, led_id):
        msg = Int32()
        msg.data = led_id
        self.led_control_pub.publish(msg)
        self.get_logger().info(f"Published LED control: {led_id}")

    def publish_display(self, plant_results):
        msg = Int32MultiArray()
        msg.data = plant_results
        self.display_pub.publish(msg)
        self.get_logger().info(f"Published display: {plant_results}")

    # -----------------------------
    # Processing
    # -----------------------------
    def process_frame(self, image):
        annotated = image.copy() #debug

        mask_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # --- YELLOW ---
        yellow_mask = cv2.inRange(mask_frame, self.yellow_lower, self.yellow_upper)
        yellow_contours, _ = cv2.findContours(
            yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        yellow_detected = False
        yellow_center_x = None

        large_yellow = [c for c in yellow_contours if cv2.contourArea(c) > MIN_AREA]
        # if large_yellow:
        #     contour = max(large_yellow, key=cv2.contourArea)
        #     x, y, w, h = cv2.boundingRect(contour)
        #     yellow_detected = True
        #     yellow_center_x = x + w // 2

        ##debug
        if large_yellow:
            contour = max(
                large_yellow,
                key=cv2.contourArea
            )

            x, y, w, h = cv2.boundingRect(contour)

            yellow_detected = True
            yellow_center_x = x + w // 2

            cv2.rectangle(
                annotated,
                (x, y),
                (x + w, y + h),
                (0, 255, 255),
                2
            )

            cv2.circle(
                annotated,
                (yellow_center_x, y + h // 2),
                5,
                (0, 255, 255),
                -1
            )

        # --- GREEN ---
        green_mask = cv2.inRange(mask_frame, self.green_lower, self.green_upper)
        green_contours, _ = cv2.findContours(
            green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        large_green = [c for c in green_contours if cv2.contourArea(c) > MIN_AREA]

        if large_green:
            contour = max(large_green, key=cv2.contourArea)
            # x, y, w, h = cv2.boundingRect(contour)
            # green_center_x = x + w // 2

            #debug
            x, y, w, h = cv2.boundingRect(contour)

            green_center_x = x + w // 2

            cv2.rectangle(
                annotated,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            cv2.circle(
                annotated,
                (green_center_x, y + h // 2),
                5,
                (0, 255, 0),
                -1
            )

            if h > HEIGHT_THRESHOLD:
                if yellow_detected:
                    if yellow_center_x < green_center_x:
                        self.publish_camera_choice(0)  # LEFT
                        #debug
                        cv2.putText(
                            annotated,
                            "LEFT",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 255, 0),
                            2
                        )
                        self.save_debug_images(
                            image,
                            yellow_mask,
                            green_mask,
                            annotated,
                            "LEFT"
                        )
                        self.publish_led_control(2)  # LED 2 ON
                        self.publish_display([0,0,1])  # Display 2 plants
                    else:
                        self.publish_camera_choice(1)  # RIGHT
                        #debug
                        cv2.putText(
                            annotated,
                            "RIGHT",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 255, 0),
                            2
                        )

                        self.save_debug_images(
                            image,
                            yellow_mask,
                            green_mask,
                            annotated,
                            "RIGHT"
                        )
                        self.publish_led_control(2)  # LED 2 ON
                        self.publish_display([0,0,1])  # Display 2 plants
                else:
                    self.publish_camera_choice(2)
                    #debug
                    cv2.putText(
                        annotated,
                        "HEALTHY",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 0, 0),
                        2
                    )

                    self.save_debug_images(
                        image,
                        yellow_mask,
                        green_mask,
                        annotated,
                        "HEALTHY"
                    )
                    self.publish_led_control(1)  # LED 1 ON
                    self.publish_display([0,1,0])  # Display 1 plant
            else:
                self.publish_camera_choice(2)
                cv2.putText(
                    annotated,
                    "SMALL_PLANT",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 255, 0),
                    2
                )

                self.save_debug_images(
                    image,
                    yellow_mask,
                    green_mask,
                    annotated,
                    "SMALL_PLANT"
                )
                self.publish_led_control(0)  # LED 0 ON
                self.publish_display([1,0,0])  # Display 0 plants
        else:
            self.publish_camera_choice(3)  # No plants detected
            #debug
            cv2.putText(
                annotated,
                "NO_PLANT",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

            self.save_debug_images(
                image,
                yellow_mask,
                green_mask,
                annotated,
                "NO_PLANT"
            )
            self.publish_led_control(-1)  # All LEDs OFF
            self.publish_display([0,0,0])  # Display no plants

    # -----------------------------
    # Callback
    # -----------------------------
    def capture_trigger_callback(self, msg):
        if not msg.data:
            return

        image = self.capture_one_frame()

        if image is None:
            self.publish_camera_choice(3)
            self.publish_led_control(-1)
            self.publish_display([0,0,0])
            return

        self.process_frame(image)

    #debug picutres:
    def save_debug_images(
        self,
        original,
        yellow_mask,
        green_mask,
        annotated,
        result_text
    ):
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        prefix = os.path.join(
            self.debug_dir,
            f"{timestamp}_{self.capture_count:04d}"
        )

        cv2.imwrite(
            f"{prefix}_original.jpg",
            original
        )

        cv2.imwrite(
            f"{prefix}_yellow_mask.jpg",
            yellow_mask
        )

        cv2.imwrite(
            f"{prefix}_green_mask.jpg",
            green_mask
        )

        cv2.imwrite(
            f"{prefix}_{result_text}.jpg",
            annotated
        )

        self.capture_count += 1

        self.get_logger().info(
            f"Saved debug images: {prefix}"
        )

    # -----------------------------
    # Cleanup
    # -----------------------------
    def destroy_node(self):
        super().destroy_node()


# -----------------------------
# Main
# -----------------------------
def main():
    rclpy.init()
    node = CameraProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()