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

MIN_AREA = 2500 # from 5000 to 3000
HEIGHT_THRESHOLD = 100 # from 150 to 125
SHOW_PREVIEW = True


# -----------------------------
# Calibration loader
# -----------------------------

def load_hsv_ranges():
    pkg_path = get_package_share_directory('identification_logic_26')
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

        # Load HSV ranges
        
        package_path = get_package_share_directory('identification_logic_26')
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
        self.led_control_pub = self.create_publisher(Int32, 'led_control', 10)
        self.display_pub = self.create_publisher(Int32MultiArray, 'display_control', 10)


        # Subscriber (matches your state machine)
        self.create_subscription(
            Bool,
            '/capture_trigger',
            self.capture_trigger_callback,
            10
        )

    def capture_one_frame(self):
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

        try:
            cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_AUTO_WB, 0) # Turn OFF auto white balance
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 1000) # Manually set white balance (tune this!)
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
            cap.set(cv2.CAP_PROP_FOCUS, 300)
            
            if not cap.isOpened():
                self.get_logger().warn("Failed to open camera")
                return None

            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn("Frame capture failed")
                return None

            cv2.imwrite("/home/corn/ros2_mr26/src/identification_logic_26/identification_logic_26/troubleshooting_pictures/lastCapture.jpg", frame)
    
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
        mask_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # --- YELLOW ---
        yellow_mask = cv2.inRange(mask_frame, self.yellow_lower, self.yellow_upper)
        yellow_contours, _ = cv2.findContours(
            yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        yellow_detected = False
        yellow_center_x = None

        large_yellow = [c for c in yellow_contours if cv2.contourArea(c) > MIN_AREA]
        if large_yellow:
            contour = max(large_yellow, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            yellow_detected = True
            yellow_center_x = x + w // 2

        # --- GREEN ---
        green_mask = cv2.inRange(mask_frame, self.green_lower, self.green_upper)
        green_contours, _ = cv2.findContours(
            green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # to see the masks
        cv2.imwrite("/home/corn/ros2_mr26/src/identification_logic_26/identification_logic_26/troubleshooting_pictures/lastYellowMaskCapture.jpg", yellow_mask)
        cv2.imwrite("/home/corn/ros2_mr26/src/identification_logic_26/identification_logic_26/troubleshooting_pictures/lastGreenMaskCapture.jpg", green_mask)

        large_green = [c for c in green_contours if cv2.contourArea(c) > MIN_AREA]

        if large_green:
            contour = max(large_green, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            green_center_x = x + w // 2

            if h > HEIGHT_THRESHOLD:
                if yellow_detected:
                    if yellow_center_x < green_center_x:
                        self.publish_led_control(2)  # LED 2 ON
                        self.publish_display([0,0,1])  # Display 2 plants
                        self.get_logger().info("Hit left")
                    else:
                        self.publish_led_control(2)  # LED 2 ON
                        self.publish_display([0,0,1])  # Display 2 plants
                        self.get_logger().info("Hit right")
                       
                else:
                    self.publish_led_control(1)  # LED 1 ON
                    self.publish_display([0,1,0])  # Display 1 plant
            else:
                self.publish_led_control(0)  # LED 0 ON
                self.publish_display([1,0,0])  # Display 0 plants
        else:
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
            self.publish_led_control(-1)  # All LEDs OFF
            self.publish_display([0,0,0])  # Display no plants
            return

        self.process_frame(image)

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