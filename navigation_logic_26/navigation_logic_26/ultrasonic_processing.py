import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, Int32


class UltrasonicProcessingNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_processing_node')

        # -------------------------
        # Parameters
        # -------------------------
        self.before_row_change_config = {
            1: (1, 12.0),
            2: (3, 12.0),
            3: (1, 12.0),
            4: (3, 12.0),
            5: (1, 12.0),
            6: (3, 12.0)
        }

        # self.row_end_config = {
        #     1: (1, 12.0),  # U2 < 26
        #     2: (3, 10.0),  # U4 < 26
        #     3: (1, 10.0),
        #     4: (3, 10.0),
        #     5: (1, 10.0),
        #     6: (3, 10.0)
        # }

        self.last_valid_distances = [0.0, 0.0, 0.0, 0.0]
        self.has_valid_reading = [False, False, False, False]


        self.declare_parameter('init_slide_threshold', 9.0) # U4 > 9.0cm to detect the wall on the left for the initial slide left in row 1
        self.init_slide_threshold = self.get_parameter(
            'init_slide_threshold').value
        
        self.declare_parameter('end_course_threshold', 2.0) # U4 <= 2.0cm to detect the wall on the left for the initial slide left in row 1
        self.end_course_threshold = self.get_parameter(
            'end_course_threshold').value

        # self.declare_parameter('init_forward_threshold', 12.0) # U4 > 12.0cm to detect the wall in front of the robot for the initial forward in row 1
        # self.init_forward_threshold = self.get_parameter(
        #     'init_forward_threshold').value

        #self.declare_parameter('row_change_start_threshold_cm', 29.0)
        #self.row_change_start_threshold = self.get_parameter(
        #    'row_change_start_threshold_cm').value

        #self.declare_parameter('row_change_arrival_threshold_cm', 59.0) 
        #self.row_change_arrival_threshold = self.get_parameter(
        #    'row_change_arrival_threshold_cm').value

        # -------------------------
        # Row-dependent side-wall configuration
        # row: (sensor_index, threshold_cm)
        # U1 → index 0, U2 → index 1, U3 → index 2, U4 → index 3
        # -------------------------
        self.before_row_follow_config = {
            1: (0, 38.0), # Row 2 → U1 > 38
            2: (0, 80.0), # Row 3 → U1 > 80
            3: (0, 114.0), # Row 4 → U1 > 90
            4: (2, 48.0), # Row 5 → U3 < 48
            5: (2, 9.0), # Row 6 → U3 < 9
        }
        
        # self.side_wall_config = {

        #     1: (2, 45.0),  # Row 2 → U3 > 59
        #     2: (2, 84.0),  # Row 3 → U3 > 89
        #     3: (0, 81.0),  # Row 4 → U1 < 84
        #     4: (0, 41.0),  # Row 5 → U1 < 54
        #     5: (2, 201.0)   # Row 6 → U1 < 24
        # }

        
        # -------------------------
        # State input
        # -------------------------
        self.current_row = 1  # 1-based indexing
        self.current_motion_state = 0  # Initialize current motion state
        
        

        # -------------------------
        # Publishers
        # -------------------------
        # self.row_end_pub = self.create_publisher(
        #     Bool, '/row_end_detected', 10)

        self.side_wall_pub = self.create_publisher(
            Bool, '/side_wall_detected', 10)

        # Separate publishers for initialization phases
        self.init_slide_wall_pub = self.create_publisher(
            Bool, '/init_slide_wall_detected', 10)

        # self.init_forward_wall_pub = self.create_publisher(
        #     Bool, '/init_forward_wall_detected', 10)

        # Serpentine row-change topics
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
        
        self.end_course_pub = self.create_publisher(
            Bool, '/end_course_detected', 10)

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

        self.get_logger().info("Ultrasonic processing node started (row-dependent thresholds)")

    # =========================
    # Callbacks
    # =========================
    def row_index_callback(self, msg):
        current_time = time.time()
        old_row = getattr(self, 'current_row', None)
        self.current_row = msg.data
        
        time_since_last = ""
        if hasattr(self, 'last_row_change') and old_row != self.current_row:
            time_since_last = f" ({current_time - self.last_row_change:.1f}s since last change)"
        
        if old_row != self.current_row:
            self.get_logger().info(f"Row changed to {self.current_row}{time_since_last}")
            self.last_row_change = current_time

    def motion_mode_callback(self, msg):
        current_time = time.time()
        old_mode = getattr(self, 'current_motion_state', None)
        self.current_motion_state = msg.data
        
        time_since_last = ""
        if hasattr(self, 'last_motion_change') and old_mode != self.current_motion_state:
            time_since_last = f" ({current_time - self.last_motion_change:.1f}s since last change)"
        
        if old_mode != self.current_motion_state:
            self.get_logger().info(f"Motion mode changed to {self.current_motion_state}{time_since_last}")
            self.last_motion_change = current_time

    def ultrasonic_callback(self, msg):
        distances = list(msg.data)

        for i in range(4):
            if distances[i] >= 0:
                self.last_valid_distances[i] = distances[i]
                self.has_valid_reading[i] = True
            elif self.has_valid_reading[i]:
                self.get_logger().warn(
                    f"U{i+1} invalid ({distances[i]}), using previous value {self.last_valid_distances[i]:.1f}"
                )
                distances[i] = self.last_valid_distances[i]

        if len(distances) != 4:
            self.get_logger().warn("Expected 4 ultrasonic distances")
            return
        
        before_row_follow_detected = False  
        # arrival_detected = False
        # before_row_follow_detected = False
        before_row_change_detected = False
        # row_end_detected = False

        # Publish filtered distances (pass-through)
        filtered = Float32MultiArray()
        filtered.data = distances
        self.filtered_pub.publish(filtered)

        # -------------------------
        # Initialization detection
        # -------------------------
        # if self.current_motion_state == 14 or self.current_motion_state == 0:  # SLOW SLIDE RIGHT init
        detected = distances[3] > self.init_slide_threshold and distances[3] >= 0  # U4 > 9
            # if detected:
            #     self.get_logger().info(
            #         f"ROW INIT SLIDE DETECTED: U1={distances[0]:.1f}cm < {self.init_slide_threshold}cm"
            #     )
        self.init_slide_wall_pub.publish(Bool(data=detected))
        self.row_change_start_pub.publish(Bool(data=False))
            # self.row_change_arrival_pub.publish(Bool(data=False))

        # if self.current_motion_state == 11:  # SLOW FORWARD init
        #     detected = distances[3] > self.init_forward_threshold  # U4 > 15
        #     if detected:
        #         self.get_logger().info(
        #             f"ROW INIT FORWARD DETECTED: U4={distances[3]:.1f}cm > {self.init_forward_threshold}cm"
        #         )
        #     self.init_forward_wall_pub.publish(Bool(data=detected))
        #     self.row_change_start_pub.publish(Bool(data=False))
        #     self.row_change_arrival_pub.publish(Bool(data=False))

        # -------------------------
        # BEFORE ROW CHANGE detection (NEW)
        # -------------------------
        #row_end_sensor = self.get_row_end_sensor_index()
        #before_distance = distances[row_end_sensor]

        
        if self.current_motion_state in (1, 2):  # row follow, so won't detecet when if the robot is turning/correcting in front of the row change, but it's better than nothing and won't cause false positives during row change
            if self.current_row in self.before_row_change_config:
                sensor_index, threshold = self.before_row_change_config[self.current_row]
                if distances [sensor_index] >= 0:
                    distance_before_row_change = distances[sensor_index]
                    before_row_change_detected = distance_before_row_change < threshold
            else:
                before_row_change_detected = False
        else:
            before_row_change_detected = False

        #self.get_logger().info(
        #    f"[DEBUG BEFORE_ROW_CHANGE] value={before_row_change_detected} | "
        #    f"motion={self.current_motion_state} | row={self.current_row}"
        #)



        self.before_row_change_pub.publish(Bool(data=before_row_change_detected))

        if before_row_change_detected:
            self.get_logger().info(
                f"U{sensor_index+1}={distance_before_row_change:.1f}cm < {threshold}cm"
            )

        # -------------------------
        # Row end detection (unchanged)
        # -------------------------
        # if self.current_motion_state in (11, 12):  # row follow, so won't detecet when if the robot is turning/correcting in front of the row change, but it's better than nothing and won't cause false positives during row change

        #     if self.current_row in self.row_end_config:
        #         sensor_index, threshold = self.row_end_config[self.current_row]
        #         distance_row_end = distances[sensor_index]
        #         row_end_detected = distance_row_end < threshold
        #     else:
        #         row_end_detected = False
        # else:
        #     row_end_detected = False

        # #self.get_logger().info(
        # #    f"[DEBUG END_ROW] value={row_end_detected} | "
        # #    f"motion={self.current_motion_state} | row={self.current_row}"
        # #)   

        # self.row_end_pub.publish(Bool(data=row_end_detected))

        # if row_end_detected:
        #    self.get_logger().info(
        #         f"ROW END DETECTED: U{sensor_index+1}={distance_row_end:.1f}cm < {threshold}cm "
        #         f"(row {self.current_row})"
        #     )

        

        # -------------------------
        # Serpentine row-change detection
        # -------------------------
        #elif self.current_motion_state in (1, 2):
            # Normal row navigation using row-dependent side wall config
        #    if self.current_row in self.side_wall_config:
        #        nav_sensor_index, nav_threshold = self.side_wall_config[self.current_row]
        #        nav_detected = distances[nav_sensor_index] > nav_threshold
        #        nav_sensor_name = f"U{nav_sensor_index+1}"
        #        if nav_detected:
        #            self.get_logger().info(
        #                f"SIDE WALL DETECTED (ROW {self.current_row}): {nav_sensor_name}={distances[nav_sensor_index]:.1f}cm > {nav_threshold}cm"
        #            )
        #        self.side_wall_pub.publish(Bool(data=nav_detected))
        #    else:
        #        self.side_wall_pub.publish(Bool(data=False))

            # Row change start is now handled by row_end_detected
        #    self.row_change_start_pub.publish(Bool(data=False))
        #    self.row_change_arrival_pub.publish(Bool(data=False))

    

        # =========================
        # ROW CHANGE ARRIVAL
        # =========================
        # if self.current_motion_state == 14:   # SLOW SLIDE RIGHT (ROW CHANGE)
        #     if self.current_row in self.side_wall_config:
        #         sensor_index, threshold = self.side_wall_config[self.current_row]

        #         if sensor_index == 0:
        #             arrival_detected = distances[sensor_index] > threshold
        #         elif sensor_index == 2:
        #             arrival_detected = distances[sensor_index] < threshold
        #         else:
        #             arrival_detected = False

        #         sensor_name = f"U{sensor_index+1}"

        #         if arrival_detected:
        #             self.get_logger().info(
        #                 f"ROW CHANGE ARRIVAL DETECTED: {sensor_name}="
        #                 f"{distances[sensor_index]:.1f}cm (threshold {threshold}cm) "
        #                 f"(row {self.current_row})"
        #             )
        #     else:
        #         arrival_detected = False

        # else:
        #     arrival_detected = False
        
        
        # self.get_logger().info(
        #     f"[ROW CHANGE ARRIVAL] value={arrival_detected} | "
        #     f"motion={self.current_motion_state} | row={self.current_row}"
        # )   

        # self.row_change_arrival_pub.publish(Bool(data=arrival_detected))
        # self.side_wall_pub.publish(Bool(data=arrival_detected))


        # =========================
        # BEFORE ROW FOLLOW
        # =========================
        
        if self.current_motion_state == 4:   # SLIDE RIGHT (ROW CHANGE)

            if self.current_row in self.before_row_follow_config:
                sensor_index, threshold = self.before_row_follow_config[self.current_row]

                if sensor_index == 2:
                    before_row_follow_detected = distances[sensor_index] < threshold and distances[sensor_index] >= 0
                elif sensor_index == 0:
                    before_row_follow_detected = distances[sensor_index] > threshold and distances[sensor_index] >= 0
                else:
                    before_row_follow_detected = False

                sensor_name = f"U{sensor_index+1}"

                if before_row_follow_detected:
                    self.get_logger().info(
                        f"BEFORE ROW FOLLOW DETECTED: {sensor_name}="
                        f"{distances[sensor_index]:.1f}cm (threshold {threshold}cm) "
                        f"(row {self.current_row})"
                    )
            else:
                before_row_follow_detected = False

        else:
            before_row_follow_detected = False
        
        # =========================
        # FINALIZE END COURSE DETECTION
        # =========================

        if self.current_motion_state == 6: # move backward
            end_course = distances[3] <= self.end_course_threshold # U4 <= 2cm
            self.end_course_pub.publish(Bool(data=end_course))

        # self.get_logger().info(
        #     f"[BEFORE ROW FOLLOW] value={before_row_follow_detected} | "
        #     f"motion={self.current_motion_state} | row={self.current_row}"
        # )   
      
        self.before_row_follow_pub.publish(Bool(data=before_row_follow_detected))
        self.row_change_start_pub.publish(Bool(data=False))

        # if arrival_detected:
        #     self.get_logger().info(
        #         f"ROW CHANGE ARRIVAL DETECTED: {sensor_name}={distances[sensor_index]:.1f}cm > {threshold}cm "
        #         f"(row {self.current_row})"
        #     )
            
        


        # else:
        # During other motion modes, publish false to avoid stale row-change state.
        # self.row_change_start_pub.publish(Bool(data=False))
        # self.row_change_arrival_pub.publish(Bool(data=False))
        # self.side_wall_pub.publish(Bool(data=False))

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
    # Sensor selection logic
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