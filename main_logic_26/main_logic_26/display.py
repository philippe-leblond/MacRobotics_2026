import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
import os

class SimpleDisplayNode(Node):

    def __init__(self):
        super().__init__('simple_display')

        # Screen size
        self.width = 800
        self.height = 480

        # Counters
        self.empty_count = 0   # E
        self.single_count = 0  # S
        self.double_count = 0  # D

        self.get_logger().info("Display node started")

        # Subscriber
        self.create_subscription(
            Int32MultiArray,
            '/display_control',
            self.display_callback,
            10
        )

        # Refresh screen
        self.timer = self.create_timer(0.1, self.update_screen)

    # -----------------------------
    # Callback (update counters)
    # -----------------------------
    def display_callback(self, msg):
        data = msg.data

        if len(data) != 3:
            self.get_logger().warn("Invalid data received")
            return

        # ONE-HOT DECODING
        if data[0] == 1:
            self.empty_count += 1
        elif data[1] == 1:
            self.single_count += 1
        elif data[2] == 1:
            self.double_count += 1

        self.get_logger().info(
            f"E:{self.empty_count} S:{self.single_count} D:{self.double_count}"
        )

    # -----------------------------
    # Display
    # -----------------------------
    def update_screen(self):
        screen = np.ones((self.height, self.width, 3), dtype=np.uint8) * 255

        # Text (dynamic!)
        text = f"S: {self.single_count}, D: {self.double_count}, E: {self.empty_count}"

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5
        thickness = 3

        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        text_x = (self.width - text_size[0]) // 2
        text_y = (self.height + text_size[1]) // 2

        cv2.putText(
            screen,
            text,
            (text_x, text_y),
            font,
            font_scale,
            (0, 0, 0),
            thickness
        )

        cv2.imshow("Touchscreen Display", screen)
        cv2.waitKey(1)


    def save_final_screen(self):
        save_dir = "/home/corn/ros2_mr26/src/main_logic_26/main_logic_26/final_report"

        screen = np.ones((self.height, self.width, 3), dtype=np.uint8) * 255

        text = f"S: {self.single_count}, D: {self.double_count}, E: {self.empty_count}"

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5
        thickness = 3

        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        text_x = (self.width - text_size[0]) // 2
        text_y = (self.height + text_size[1]) // 2

        cv2.putText(
            screen,
            text,
            (text_x, text_y),
            font,
            font_scale,
            (0, 0, 0),
            thickness
        )

        filename = os.path.join(save_dir, "final_display.png")
        cv2.imwrite(filename, screen)

        self.get_logger().info(f"Saved final display to {filename}")


def main():
    rclpy.init()
    node = SimpleDisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_final_screen()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()