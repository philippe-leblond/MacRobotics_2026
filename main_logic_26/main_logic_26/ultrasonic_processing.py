import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool, Int32


class UltrasonicProcessingNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_processing_node')

        # -------------------------
        # PID Plant Alignment
        # -------------------------
        self.kp = 0.03
        self.pid_deadband = 1.0

        # -------------------------
        # Parameters
        # -------------------------
        
        self.declare_parameter('init_slide_1_threshold', 203.0) # u3 < 203
        self.init_slide_1_threshold = self.get_parameter('init_slide_1_threshold').value

        self.declare_parameter('init_slide_2_threshold', 36.0) # U1 > 36
        self.init_slide_2_threshold = self.get_parameter('init_slide_2_threshold').value

        # -------------------------
        # Plant detection config
        # -------------------------

        self.row_plants = {

            # Sensor index, operator, between_dashes, pid_target
            1: [
                (3, ">", 42), # 42 0
                (3, ">", 65), # 65 1
                (1, "<", 114), # 114 2 
                (1, "<", 87), # 87 3
                (1, "<", 61), # 61 4 
                (1, "<", 36), # 36 5
            ],
            2: [
                (1, ">", 39), # 39
                (1, ">", 61), # 61
                (1, ">", 87), # 87
                (1, ">", 113), # 113
                (3, "<", 64), # 64
                (3, "<", 38), # 38
            ],
            3: [
                (3, ">", 42), # 42 0
                (3, ">", 65), # 65 1
                (1, "<", 114), # 114 2 
                (1, "<", 87), # 87 3
                (1, "<", 61), # 61 4 
                (1, "<", 36), # 36 5
            ],
            4: [
                (1, ">", 39), # 39
                (1, ">", 61), # 61
                (1, ">", 87), # 87
                (1, ">", 113), # 113
                (3, "<", 64), # 64
                (3, "<", 38), # 38
            ],
            5: [
                (3, ">", 42), # 42 0
                (3, ">", 65), # 65 1
                (1, "<", 114), # 114 2 
                (1, "<", 87), # 87 3
                (1, "<", 61), # 61 4 
                (1, "<", 36), # 36 5
            ],
        }


        # -------------------------
        # Before row change config and end of the row config 
        # -------------------------
        self.before_row_change_config = {
            1: (1, 19.0),
            2: (3, 18.0),
            3: (1, 18.0),
            4: (3, 18.0),
            5: (1, 18.0),
            6: (3, 10.0)
        }

        # self.row_end_config = {
        #     1: (1, 12.0),  # U2 < 26
        #     2: (3, 10.0),  # U4 < 26
        #     3: (1, 10.0),
        #     4: (3, 10.0),
        #     5: (1, 10.0),
        #     6: (3, 10.0)
        # }

        # -------------------------
        # Row-dependent side-wall configuration
        # row: (sensor_index, threshold_cm)
        # U1 → index 0, U2 → index 1, U3 → index 2, U4 → index 3
        # -------------------------
        self.before_row_follow_config = {
            1: (2, 39.0), # Row 2 → U3 > 54
            2: (2, 69.0), # Row 3 → U3 > 84
            3: (0, 94.0), # Row 4 → U1 < 86
            4: (0, 48.0), # Row 5 → U1 < 56
            5: (0, 10.0), # Row 6 → U1 < 26
        }
        
        self.current_plant = 0
        self.plant_latched = False

        self.between_dashes = True

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
            Bool, '/row_end', 10)

        self.side_wall_pub = self.create_publisher(
            Bool, '/side_wall_detected', 10)
        
        # self.row_change_start_pub = self.create_publisher(
        #     Bool, '/row_change_start_detected', 10)

        self.init_slide_wall_1_pub = self.create_publisher(Bool, '/init_slide_wall_1_detected', 10)
        self.init_slide_wall_2_pub = self.create_publisher(Bool, '/init_slide_wall_2_detected', 10)
        # self.init_slide_wall_3_pub = self.create_publisher(Bool, '/init_slide_wall_3_detected', 10)

        self.init_forward_wall_1_pub = self.create_publisher(Bool, '/init_forward_wall_1_detected', 10)
        #self.init_forward_wall_2_pub = self.create_publisher(Bool, '/init_forward_wall_2_detected', 10)


        #self.row_change_arrival_pub = self.create_publisher(
        #    Bool, '/row_change_arrival_detected', 10)

        self.between_dashes_pub = self.create_publisher(
            Bool, '/between_dashes', 10)

        self.row_change_start_pub = self.create_publisher(
            Bool, '/row_change_start_detected', 10)

        # self.row_change_arrival_pub = self.create_publisher(
        #     Bool, '/row_change_arrival_detected', 10)

        self.filtered_pub = self.create_publisher(
            Float32MultiArray, '/ultrasonic/distances_filtered', 10)
        
        self.before_row_change_pub = self.create_publisher(
            Bool, '/before_row_change_detected', 10)

        self.before_row_follow_pub = self.create_publisher(
            Bool, '/before_row_follow_detected', 10)
        
        self.plant_pid_pub = self.create_publisher(
            Float32, '/plant_pid_output', 10)

        self.plant_position_reached_pub = self.create_publisher(
            Bool,'/plant_position_reached', 10)

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

        self.get_logger().warn(
            f"[PLANT INDEX CB] received={msg.data}, previous={self.current_plant}, latched={self.plant_latched}"
        )

        if msg.data == 0:
            self.get_logger().warn("🔄 Resetting latch for new row")
            self.plant_latched = False
            

        elif msg.data != self.current_plant:
            self.get_logger().warn("🔄 Resetting latch and between dashes for plant change")
            self.plant_latched = False
            self.between_dashes = False

        self.current_plant = msg.data

    def row_index_callback(self, msg):
        if msg.data != self.current_row:
            self.get_logger().warn("🔄 Row changed → resetting latch")
            self.plant_latched = False

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
                f"latched={self.plant_latched} "
                f"between_dashes={self.between_dashes}"
            )
            self._last_plant_log_time = now


        # Pass-through
        self.filtered_pub.publish(Float32MultiArray(data=distances))

        before_row_follow_detected = False  
        # arrival_detected = False
        # before_row_follow_detected = False
        before_row_change_detected = False
        # row_end_detected = False

        # -------------------------
        # Row end detection
        # -------------------------
        #row_end_sensor = self.get_row_end_sensor_index()
        #row_end_detected = distances[row_end_sensor] < self.row_end_threshold
        #self.row_end_pub.publish(Bool(data=row_end_detected))

        # -------------------------
        # Init detection
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


        # -------------------------
        # Between Dashes 
        # -------------------------
        # Select correct plant table based on row
        table = self.row_plants.get(self.current_row)   

        if table is not None and self.current_plant < len(table):
            index = max(0, self.current_plant -1)
            sensor, op, threshold = table[index]
            distance = distances[sensor]

            if not self.plant_latched and not self.between_dashes:
                if op == ">" and distance > threshold:
                    self.between_dashes = True
                if op == "<" and distance < threshold:
                    self.between_dashes = True

                if self.between_dashes:
                    self.plant_latched = True
                    self.get_logger().info(
                        f"PLANT DETECTED: row={self.current_row} "
                        f"plant={self.current_plant + 1} "
                        f"U{sensor + 1}={distance:.1f}cm "
                        f"{op} {threshold}cm"
                    )

        self.between_dashes_pub.publish(Bool(data=self.between_dashes))
        # -------------------------
        # BEFORE ROW CHANGE detection (NEW)
        # -------------------------

        
        if self.current_motion_state in (1, 2):  # row follow, so won't detecet when if the robot is turning/correcting in front of the row change, but it's better than nothing and won't cause false positives during row change
            if self.current_row in self.before_row_change_config:
                sensor_index, threshold = self.before_row_change_config[self.current_row]
                distance_before_row_change = distances[sensor_index]
                before_row_change_detected = distance_before_row_change < threshold
            else:
                before_row_change_detected = False
        else:
            before_row_change_detected = False

        self.before_row_change_pub.publish(Bool(data=before_row_change_detected))

        if before_row_change_detected:
            self.get_logger().info(
                f"U{sensor_index+1}={distance_before_row_change:.1f}cm < {threshold}cm"
            )

        # =========================
        # BEFORE ROW FOLLOW
        # =========================
        
        if self.current_motion_state == 4 or 18:   # SLIDE RIGHT (ROW CHANGE)

            if self.current_row in self.before_row_follow_config:
                sensor_index, threshold = self.before_row_follow_config[self.current_row]

                if sensor_index == 2:
                    before_row_follow_detected = distances[sensor_index] > threshold
                elif sensor_index == 0:
                    before_row_follow_detected = distances[sensor_index] < threshold
                else:
                    before_row_follow_detected = False
            else:
                before_row_follow_detected = False

        else:
            before_row_follow_detected = False
      
        self.before_row_follow_pub.publish(Bool(data=before_row_follow_detected))
        self.row_change_start_pub.publish(Bool(data=False))


        # Log current distances periodically (every ~5 seconds to avoid spam)
        current_time = time.time()
        if not hasattr(self, 'last_distance_log') or current_time - self.last_distance_log > 5.0:
            self.get_logger().info(
                f"Ultrasonic distances: U1={distances[0]:.1f}cm U2={distances[1]:.1f}cm "
                f"U3={distances[2]:.1f}cm U4={distances[3]:.1f}cm "
                f"(row {self.current_row}, motion {self.current_motion_state})"
            )
            self.last_distance_log = current_time

    # =========================
    # HELPERS
    # =========================
    def get_row_end_sensor_index(self):
        """
        Rows 1,3 and 5 → U2 (index 1) # for the lateral distance to the wall in front of the robot
        Rows 2,4 and 6 → U4 (index 3) # for the lateral distance to the wall in the back of the robot
        """
        if self.current_row % 2 == 1: # odd rows (1,3,5)
            return 1  # U2
        elif self.current_row % 2 == 0: # even rows (2,4,6)
            return 3  # U4
        




def main():
    rclpy.init()
    node = UltrasonicProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()