import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool, Int32
from enum import Enum
import time


class LineMode(Enum):
    ROW_FOLLOW_ODD = 0
    SLOW_ROW_FOLLOW_ODD = 1
    ROW_FOLLOW_EVEN = 2
    SLOW_ROW_FOLLOW_EVEN = 3
    LEFT_ROW_CHANGE = 4
    RIGHT_ROW_CHANGE = 5
    NO_LINE_SENSORS = 6


class LineProcessingNode(Node):

    def __init__(self):
        super().__init__('line_processing_node')

        self.mode = LineMode.NO_LINE_SENSORS
        # Store previous sensor states
        self.prev_sensors = [0, 0, 0, 0]

        # Publishers
        self.detected_pub = self.create_publisher(
            Bool, '/line_detected', 10)

        self.forward_pair_pub = self.create_publisher(
            Bool, '/forward_pair_black', 10)

        self.backward_pair_pub = self.create_publisher(
            Bool, '/backward_pair_black', 10)

        self.direction_pub = self.create_publisher(
            Int32, '/line_direction', 10)
        
        self.falling_edge_pub = self.create_publisher(
            Int32MultiArray, '/line_falling_edges', 10)

        # Subscriptions
        self.create_subscription(
            Int32MultiArray,
            '/line_sensors/raw',
            self.line_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/line_mode',
            self.mode_callback,
            10
        )

        self._last_log = 0.0
        self._last_mode_log = 0.0
        self.get_logger().info(" Line processing node started")

    # =========================
    # Callbacks
    # =========================
    def mode_callback(self, msg):
        try:
            self.mode = LineMode(msg.data)
            now = time.monotonic()
            if now - self._last_mode_log >= 1.0:
                self.get_logger().info(f"Line mode → {self.mode.name}")
                self._last_mode_log = now
        except ValueError:
            self.get_logger().warn("Invalid line mode received")

    def line_callback(self, msg):
        sensors = msg.data
        if len(sensors) != 4:
            return

        active = self.get_active_indices(sensors)
        detected = len(active) > 0
        self.detected_pub.publish(Bool(data=detected))

        self.forward_pair_pub.publish(
            Bool(data=(0 in active and 1 in active)))

        self.backward_pair_pub.publish(
            Bool(data=(2 in active and 3 in active)))

        direction = self.compute_direction(active)
        self.direction_pub.publish(Int32(data=direction))

        # Detect falling edges (1 -> 0)
        falling_edges = [0, 0, 0, 0] # CAN ADD THRESHOLDS IF NEEDED IF THE LINE SENSORS ARE FLASHING 

        for i in range(4):
            if self.prev_sensors[i] == 1 and sensors[i] == 0:
                falling_edges[i] = 1  # TRUE only once

        # Publish edges
        self.falling_edge_pub.publish(Int32MultiArray(data=falling_edges))

        # Update previous values
        self.prev_sensors = list(sensors)

        #now = time.monotonic()
        #if now - self._last_log > 1.0:
        #    self.get_logger().info(
        #        f"sensors={sensors} mode={self.mode.name} active={active} dir={direction}"
        #    )
        #    self._last_log = now

    # =========================
    # Logic
    # =========================
    def get_active_indices(self, sensors):
        if self.mode in (LineMode.ROW_FOLLOW_ODD, LineMode.SLOW_ROW_FOLLOW_ODD):
            indices = [0, 1]
        elif self.mode in (LineMode.ROW_FOLLOW_EVEN, LineMode.SLOW_ROW_FOLLOW_EVEN):
            indices = [2, 3]
        elif self.mode == LineMode.RIGHT_ROW_CHANGE:
            indices = [1, 2]
        elif self.mode == LineMode.LEFT_ROW_CHANGE:
            indices = [0, 3]
        else:
            indices = []

        return [i for i in indices if sensors[i] == 1]

    def compute_direction(self, active):
        if not active:
            return 0

        s = active[0]

        if self.mode in (LineMode.ROW_FOLLOW_ODD, LineMode.SLOW_ROW_FOLLOW_ODD):
            if 0 in active and 1 in active:
                return 0
            return 1 if s == 0 else -1 # if the first sensor is active, we are on the left side of the line and need to turn left, otherwise we are on the right side of the line and need to turn right

        if self.mode in (LineMode.ROW_FOLLOW_EVEN, LineMode.SLOW_ROW_FOLLOW_EVEN):
            if 2 in active and 3 in active:
                return 0
            return 1 if s == 2 else -1

        if self.mode == LineMode.RIGHT_ROW_CHANGE:
            if 1 in active and 2 in active:
                return 0
            return 1 if s == 1 else -1

        if self.mode == LineMode.LEFT_ROW_CHANGE:
            if 0 in active and 3 in active:
                return 0
            return 1 if s == 0 else -1

        return 0


def main():
    rclpy.init()
    node = LineProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()