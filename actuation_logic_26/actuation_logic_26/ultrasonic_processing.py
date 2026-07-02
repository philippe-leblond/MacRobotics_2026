import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool, Int32


class UltrasonicProcessingNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_processing_node')

        # -------------------------
        # Parameters
        # -------------------------
        self.kp = 0.03
        self.pid_deadband = 1.0 # try with 2.0 than maybe lower after

        self.declare_parameter('row_end_threshold_cm', 20.0) # For detecting the wall before making a turn. It's the same threshold for both rows. Might need to change it in testing
        self.row_end_threshold = self.get_parameter(
            'row_end_threshold_cm').value
        
        self.declare_parameter('init_slide_1_threshold', 203.0) # u3 < 203
        self.init_slide_1_threshold = self.get_parameter('init_slide_1_threshold').value

        self.declare_parameter('init_slide_2_threshold', 36.0) # U1 > 36
        self.init_slide_2_threshold = self.get_parameter('init_slide_2_threshold').value

        # self.declare_parameter('init_slide_3_threshold', 38.0) # u1 > 43
        # self.init_slide_3_threshold = self.get_parameter('init_slide_3_threshold').value

        # self.declare_parameter('init_forward_1_threshold', 9.0) # U4 > 9
        # self.init_forward_1_threshold = self.get_parameter('init_forward_1_threshold').value

        #self.declare_parameter('init_forward_2_threshold', 15.0)
        #self.init_forward_2_threshold = self.get_parameter('init_forward_2_threshold').value

        # -------------------------
        # Side-wall configuration (ROW 1 <-> ROW 2 only)
        # -------------------------
        # Row 1 -> Row 2 : SLIDE LEFT  → U1 > threshold
        # Row 2 -> Row 1 : SLIDE RIGHT → U3 < threshold
        #self.side_wall_config = {
        #    1: (0, 59.0),  # U1 > 59 # from row 1 to row 2 we look at U1 and expect it to be greater than 59 cm
        #    2: (0, 25.0),  # U1 < 25 # from row 2 to row 1 we look at U1 and expect it to be less than 25 cm
        #}

        # -------------------------
        # Plant detection config
        # -------------------------
        self.row_odd_plants = [

            # sensor, detection_op, detect_threshold, pid_target

            (3, ">", 37, 40),
            (3, ">", 57, 60),
            (1, "<", 103, 100),
            (1, "<", 92, 89),
            (1, "<", 66, 63),
            (1, "<", 42, 35),
        ]

        self.row_even_plants = [
            (3, ">", 37, 40),
            (3, ">", 57, 60),
            (1, "<", 113, 110),
            (1, "<", 92, 89),
            (1, "<", 66, 63),
            (1, "<", 41, 38),
        ]

        self.current_plant = 0
        self.plant_latched = False

        # -------------------------
        # State input
        # -------------------------
        self.current_row = 1
        self.current_motion_state = 0


        self._last_plant_log_time = 0.0 # logging purpose only

        # -------------------------
        # Publishers
        # -------------------------
        self.row_end_pub = self.create_publisher(
            Bool, '/row_end_detected', 10)
        
        self.plant_pid_pub = self.create_publisher(Float32, '/plant_pid_output', 10)



        #self.side_wall_pub = self.create_publisher(
        #    Bool, '/side_wall_detected', 10)

        self.init_slide_wall_1_pub = self.create_publisher(Bool, '/init_slide_wall_1_detected', 10)
        self.init_slide_wall_2_pub = self.create_publisher(Bool, '/init_slide_wall_2_detected', 10)
        # self.init_slide_wall_3_pub = self.create_publisher(Bool, '/init_slide_wall_3_detected', 10)

        self.init_forward_wall_1_pub = self.create_publisher(Bool, '/init_forward_wall_1_detected', 10)
        #self.init_forward_wall_2_pub = self.create_publisher(Bool, '/init_forward_wall_2_detected', 10)


        #self.row_change_arrival_pub = self.create_publisher(
        #    Bool, '/row_change_arrival_detected', 10)

        self.plant_detected_pub = self.create_publisher(
            Bool, '/plant_detected', 10)

        self.filtered_pub = self.create_publisher(
            Float32MultiArray,
            '/ultrasonic/distances_filtered',
            10
        )

        self.plant_position_reached_pub = self.create_publisher(
            Bool,
            '/plant_position_reached',
            10
        )

        # -------------------------
        # Subscriptions
        # -------------------------
        self.create_subscription(
            Float32MultiArray,
            '/ultrasonic/distances',
            self.ultrasonic_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/row_index',
            self.row_index_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/motion_mode',
            self.motion_mode_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/plant_index',
            self.plant_index_callback,
            10
        )

        self.get_logger().info("Ultrasonic processing node ready")

    # =========================
    # Callbacks
    # =========================
    def plant_index_callback(self, msg):
        if msg.data != self.current_plant:
            self.get_logger().info("Plant index changed → resetting latch")
            self.plant_latched = False

        self.current_plant = msg.data


    def row_index_callback(self, msg):
        if msg.data != self.current_row:
            self.get_logger().info("Row changed → resetting plant latch")
            self.plant_latched = False
            self.current_plant = 0   # optional safety reset

        self.current_row = msg.data

    def motion_mode_callback(self, msg):
        self.current_motion_state = msg.data

    # =========================
    # Main processing
    # =========================
    def ultrasonic_callback(self, msg):
        distances = msg.data
        if len(distances) != 4:
            return
        
        # Debug row + motion state
        now = time.monotonic()
        if now - self._last_plant_log_time >= 1.0:
            self.get_logger().info(
                f"[ULTRASONIC] current_row={self.current_row} "
                f"current_plant={self.current_plant} "
                f"motion={self.current_motion_state} "
                f"latched={self.plant_latched}"
            )
            self._last_plant_log_time = now


        # Pass-through
        self.filtered_pub.publish(Float32MultiArray(data=distances))

        # -------------------------
        # Row end detection
        # -------------------------
        row_end_detected = distances[1] < self.row_end_threshold
        self.row_end_pub.publish(Bool(data=row_end_detected))

        # -------------------------
        # Init detection
        # -------------------------
        # -------------------------
        # Init detection (independent of motion state)
        # -------------------------

        # INIT SLIDE 1 (wall detection)
        det_slide_1 = distances[2] < self.init_slide_1_threshold

        # INIT FORWARD
        # det_forward_1 = distances[3] > self.init_forward_1_threshold

        # INIT SLIDE 2 (your U1 > 36 logic)
        det_slide_2 = distances[0] > self.init_slide_2_threshold


        # Publish ALL detections always
        self.init_slide_wall_1_pub.publish(Bool(data=det_slide_1))
        # self.init_forward_wall_1_pub.publish(Bool(data=det_forward_1))
        self.init_slide_wall_2_pub.publish(Bool(data=det_slide_2))

        # if self.current_motion_state == 14:  # SLOW SLIDE RIGHT init
        #     detected = distances[2] < self.init_slide_1_threshold
        #     self.init_slide_wall_1_pub.publish(Bool(data=detected))
        #     # detected = distances[2] > self.init_slide_3_threshold # Before going in the row follow
        #     # self.init_slide_wall_3_pub.publish(Bool(data=detected))
        # elif self.current_motion_state == 5:  # FORWARD init
        #     detected = distances[3] > self.init_forward_1_threshold
        #     self.init_forward_wall_1_pub.publish(Bool(data=detected))
        # elif self.current_motion_state == 4:  # LINE_FOLLOW_SLIDE_RIGHT init
        #     detected = distances[0] > self.init_slide_2_threshold
        #     self.init_slide_wall_2_pub.publish(Bool(data=detected))
        # # elif self.current_motion_state == 14:  # SLOW SLIDE RIGHT init
        # #     detected = distances[3] > self.init_slide_3_threshold
        # #     self.init_slide_wall_3_pub.publish(Bool(data=detected))
        # #elif self.current_motion_state == 5:  # FORWARD init
        # #    detected = distances[3] > self.init_forward_2_threshold
        # #    self.init_forward_wall_2_pub.publish(Bool(data=detected))
        # else:
        #     self.init_slide_wall_1_pub.publish(Bool(data=False))
        #     self.init_forward_wall_1_pub.publish(Bool(data=False))
        #     self.init_slide_wall_2_pub.publish(Bool(data=False))
            # self.init_slide_wall_3_pub.publish(Bool(data=False))
            #self.init_forward_wall_2_pub.publish(Bool(data=False))

        # -------------------------
        # Row change arrival (LEFT & RIGHT)
        # -------------------------
        #arrival_detected = False

        #if self.current_motion_state in (3, 4) and self.current_row in self.side_wall_config:
        #    sensor, threshold = self.side_wall_config[self.current_row]
        #    if sensor == 0:
        #        arrival_detected = distances[sensor] > threshold
        #    elif sensor == 2:
        #        arrival_detected = distances[sensor] < threshold

        #self.row_change_arrival_pub.publish(Bool(data=arrival_detected))
        #self.side_wall_pub.publish(Bool(data=arrival_detected))

        # -------------------------
        # Plant detection (ONLY in slow modes)
        # -------------------------
        plant_detected = False

        # Select correct plant table based on row + motion direction
        if self.current_row % 2 and self.current_motion_state == 11: # maybe need to remove the added condition of the current motion state when the camera vision will be implemented
            table = self.row_odd_plants
        elif self.current_row % 2 == 0 and self.current_motion_state == 11:  # maybe need to remove the added condition of the current motion state when the camera vision will be implemented
            table = self.row_even_plants
        else:
            table = None  # invalid combination → do nothing

        if table is not None and self.current_plant < len(table):
            sensor, op, threshold, pid_target = table[self.current_plant]
            distance = distances[sensor]

            if not self.plant_latched:
                if op == ">" and distance > threshold:
                    plant_detected = True
                elif op == "<" and distance < threshold:
                    plant_detected = True

                if plant_detected:
                    self.plant_latched = True
                    self.get_logger().info(
                        f"PLANT DETECTED: row={self.current_row} "
                        f"DETECTED: motion={self.current_motion_state} "
                        f"plant={self.current_plant + 1} "
                        f"U{sensor + 1}={distance:.1f}cm "
                        f"{op} {threshold}cm"
                    )

        self.plant_detected_pub.publish(Bool(data=plant_detected))

        # -------------------------
        # PID plant allignement (After Plant Detection)
        # -------------------------

        if self.current_motion_state == 15:  # PID_CONTROL mode

            config = self.get_current_plant_config()

            if config is not None:

                sensor, op, threshold, pid_target = config

                distance = distances[sensor]

                pid_output = self.compute_p_output(
                    distance,
                    pid_target,
                    op
                )

                error = self.compute_plant_error(
                    distance,
                    pid_target,
                    op
                )

                position_reached = abs(error) < self.pid_deadband

                self.plant_position_reached_pub.publish(
                    Bool(data=position_reached)
                )

                self.plant_pid_pub.publish(
                    Float32(data=pid_output)
                )
    # =========================
    # Helpers
    # =========================
    def get_current_plant_config(self):

        if self.current_row % 2:
            table = self.row_odd_plants
        else:
            table = self.row_even_plants

        if self.current_plant >= len(table):
            return None

        return table[self.current_plant]
    
    def compute_p_output(self, distance, target, op):

        error = self.compute_plant_error(
            distance,
            target,
            op
        )

        if abs(error) < self.pid_deadband:
            return 0.0

        return max(
            -0.25,
            min(0.25, self.kp * error)
        )


    def compute_plant_error(self, distance, target, op):

        if op == ">":
            return target - distance

        elif op == "<":
            return distance - target

        return 0.0

def main():
    rclpy.init()
    node = UltrasonicProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()