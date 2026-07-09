import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Int32MultiArray
from enum import Enum


class RobotState(Enum):
    INIT_SLIDE_LEFT = 0
    INIT_FORWARD = 1
    ROW_FOLLOW = 2
    BEFORE_ROW_CHANGE = 3   
    ROW_CHANGE = 4
    BEFORE_ROW_FOLLOW = 5
    FINISHED = 6


class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine_node')

        # -------------------------
        # State
        # -------------------------
        self.state = RobotState.INIT_SLIDE_LEFT
        self.current_row = 1
        self.max_rows = 6
        self.row_change_latched = False  # To prevent multiple detections during a single row change

        # self.row_end_detected = False
        self.side_wall_detected = False
        self.line_detected = False
        self.init_slide_wall_detected = False
        # self.init_slide_wall_detected_time = None
        # self.init_forward_wall_detected = False
        # self.row_change_arrival_detected = False
        self.before_row_change_detected = False
        self.before_row_follow_detected = False
        # self.init_slide_wall_detected_time = None
        self.falling_edges = [0,0,0,0]


        # -------------------------
        # Publishers
        # -------------------------
        self.motion_pub = self.create_publisher(
            Int32, '/motion_mode', 10)

        self.line_mode_pub = self.create_publisher(
            Int32, '/line_mode', 10)

        self.row_pub = self.create_publisher(
            Int32, '/row_index', 10)

        # -------------------------
        # Subscriptions
        # -------------------------
        # self.create_subscription(
        #     Bool, '/row_end_detected', self.row_end_cb, 10)

        self.create_subscription(
            Bool, '/side_wall_detected', self.side_wall_cb, 10)

        self.create_subscription(
            Bool, '/init_slide_wall_detected', self.init_slide_wall_cb, 10)

        # self.create_subscription(
        #     Bool, '/init_forward_wall_detected', self.init_forward_wall_cb, 10)

        # self.create_subscription(
        #     Bool, '/row_change_arrival_detected', self.row_change_arrival_cb, 10)

        self.create_subscription(
            Bool, '/line_detected', self.line_detected_cb, 10)

        self.create_subscription(
            Bool, '/before_row_change_detected', self.before_row_change_cb, 10)

        self.create_subscription(
            Bool, '/before_row_follow_detected', self.before_row_follow_cb, 10)

        self.create_subscription(
            Int32MultiArray, '/line_falling_edges', self.falling_edges_cb, 10)
        
        self.create_subscription(
            Bool, '/end_course_detected', self.end_course_cb, 10)

        # -------------------------
        # Timer
        # -------------------------
        self.timer = self.create_timer(0.1, self.update)

        # self.side_wall_log_period = 1  # seconds -> 1 Hz
        # self.next_side_wall_log_time = time.time()

        self.before_row_change_log_period = 1.0  # seconds
        self.next_before_row_change_log_time = time.time()

        self.before_row_follow_log_period = 1.0  # seconds
        self.next_before_row_follow_log_time = time.time()


        self.get_logger().info("State machine started")

        
        self.init_start_time = 0.0  # Track when we entered the current state for timeout purposes


        # -------------------------
        # Serial (ttyUSB1)
        # -------------------------
        self.ser = None
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            self.get_logger().info("Opened serial port /dev/ttyUSB1")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")



    # =========================
    # Callbacks
    # =========================
    # def row_end_cb(self, msg):
    #     self.row_end_detected = msg.data

    def side_wall_cb(self, msg):
        self.side_wall_detected = msg.data

    def init_slide_wall_cb(self, msg):
        self.init_slide_wall_detected = msg.data

    # def init_forward_wall_cb(self, msg):
    #     self.init_forward_wall_detected = msg.data

    # def row_change_arrival_cb(self, msg):
    #     self.row_change_arrival_detected = msg.data

    def before_row_change_cb(self, msg):
        self.before_row_change_detected = msg.data

    def before_row_follow_cb(self, msg):
        self.before_row_follow_detected = msg.data

    def line_detected_cb(self, msg):
        self.line_detected = msg.data
    
    def falling_edges_cb(self, msg):
        self.falling_edges = msg.data
    
    def end_course_cb(self, msg):
        self.end_course = msg.data


    # =========================
    # Main FSM logic
    # =========================
    def update(self):

        current_time = time.time()

        # Publish side_wall_detected state at 1 Hz
        #if current_time >= self.next_side_wall_log_time:
        #    self.get_logger().info(
        #        f"side_wall_detected state: {self.side_wall_detected}"
        #    )
        #    self.next_side_wall_log_time = current_time + self.side_wall_log_period

        # Always publish row index
        self.row_pub.publish(Int32(data=self.current_row))

        # Reset row change latch if not in ROW_CHANGE state
        if self.state != RobotState.ROW_CHANGE:
                self.row_change_latched = False


        # ===== INIT: SLIDE RIGHT =====
        if self.state == RobotState.INIT_SLIDE_LEFT:
            if current_time - self.init_start_time < 2.0: # 1 seconds timeout for initialization
                self.motion_pub.publish(Int32(data=0))  # STOP
              # Short delay to read well the ultrasonic sesnsors at the beginning
            else:
                self.line_mode_pub.publish(Int32(data=6))   # NO LINE DETECTION
                self.motion_pub.publish(Int32(data=11))     # MOVE FORWARD

            
            if self.init_slide_wall_detected:
                    self.get_logger().info("Init step 1 complete")
                    #self.motion_pub.publish(Int32(data=0))
                    self.state = RobotState.INIT_FORWARD

                #self.init_slide_wall_detected_time = None  # Reset for next use

        # ===== INIT: MOVE FORWARD =====
        elif self.state == RobotState.INIT_FORWARD:
            self.motion_pub.publish(Int32(data=8))  # SLIDE RIGHT # need to try it 
            self.line_mode_pub.publish(Int32(data=6))  # NO LINE DETECTION

            if self.falling_edges[1] == 1:  # Assuming L2 is the sensor
                self.falling_edges = [0,0,0,0] # reset the falling edge
                self.get_logger().info("Initialization complete")
                self.state = RobotState.ROW_FOLLOW

        # -------- ROW FOLLOW --------
        elif self.state == RobotState.ROW_FOLLOW:
            if self.current_row % 2 == 1:
                self.line_mode_pub.publish(Int32(data=0))  # ROW_FOLLOW_ODD
                self.motion_pub.publish(Int32(data=1))     # LINE FOLLOW FORWARD
            else:
                self.line_mode_pub.publish(Int32(data=2))  # ROW_FOLLOW_EVEN
                self.motion_pub.publish(Int32(data=2))     # LINE FOLLOW BACKWARD

            #if self.row_end_detected and not self.side_wall_detected:
            if self.before_row_change_detected:

                self.get_logger().info(
                    f"Before row change detected on row {self.current_row}"
                )
                #self.state = RobotState.ROW_CHANGE
                self.state = RobotState.BEFORE_ROW_CHANGE

            # Debug BEFORE_ROW_CHANGE flag at 1 Hz
            #if current_time >= self.next_before_row_change_log_time:
            #    self.get_logger().info(
            #        f"[DEBUG FSM] before_row_change_detected={self.before_row_change_detected}"
            #    )
            #    self.next_before_row_change_log_time = current_time + self.before_row_change_log_period

        # -------- BEFORE ROW CHANGE --------
        elif self.state == RobotState.BEFORE_ROW_CHANGE:

            if self.current_row % 2 == 1: # ODD
                self.line_mode_pub.publish(Int32(data=6))  # NO_LINE_SENSORS
                self.motion_pub.publish(Int32(data=11))     # SLOW FORWARD
                if self.falling_edges[1] == 1:  # and not self.row_change_latched ##Assuming L2 is the sensor
                    self.falling_edges = [0,0,0,0] # reset the falling edge
                    self.get_logger().info("Re-enabling line following")
                    # self.current_row += 1
                    # self.row_change_latched = True
                    # self.get_logger().info(f"Reached next row {self.current_row} after row change")
                    self.state = RobotState.ROW_CHANGE

            else: # EVEN
                self.line_mode_pub.publish(Int32(data=6))  # NO_LINE_SENSORS
                self.motion_pub.publish(Int32(data=12))     # SLOW BACKWARD      
                # To enter in the finishing state
                if self.falling_edges[3] == 1: # and not self.row_change_latched ##Assuming L4 is the sensor
                    # self.current_row += 1
                    # self.row_change_latched = True
                    self.falling_edges = [0,0,0,0] # reset the falling edge
                    self.get_logger().info("Before row change complete")

                    self.get_logger().info(
                        f"Reached next row {self.current_row} after row change"
                    )

                    self.get_logger().info(
                        f"current_row={self.current_row} ({type(self.current_row)}) "
                        f"max_rows={self.max_rows} ({type(self.max_rows)})"
                    )
                    # Decide next state
                    if self.current_row >= self.max_rows:
                        self.get_logger().info("going to finish")
                        self.state = RobotState.FINISHED
                    else:
                        #self.state = RobotState.ROW_FOLLOW
                        self.get_logger().info("Re-enabling line following")
                        self.state = RobotState.ROW_CHANGE


        # -------- ROW CHANGE --------
        elif self.state == RobotState.ROW_CHANGE:

            # From forward row → go RIGHT
            if self.current_row % 2 == 1: # ODD
                self.motion_pub.publish(Int32(data=18)) # RIGHT ROW CHANGE
                self.line_mode_pub.publish(Int32(data=5)) # SLIDE RIGHT

                if self.before_row_follow_detected:  # reuse a valid sensor
                    self.get_logger().info("Row change complete odd")
                    self.state = RobotState.BEFORE_ROW_FOLLOW
            
            elif self.current_row % 2 == 0: # EVEN
                self.motion_pub.publish(Int32(data=18)) # RIGHT ROW CHANGE
                self.line_mode_pub.publish(Int32(data=5)) # SLIDE RIGHT

                if self.before_row_follow_detected:  # reuse a valid sensor
                    self.get_logger().info("Row change complete even")
                    self.state = RobotState.BEFORE_ROW_FOLLOW
            

            # Debug BEFORE_ROW_FOLLOW flag at 1 Hz
            # if current_time >= self.next_before_row_follow_log_time:
            #     self.get_logger().info(
            #         f"[DEBUG FSM] before_row_follow_detected={self.before_row_follow_detected}"
            #     )
            #     self.next_before_row_follow_log_time = (
            #         current_time + self.before_row_follow_log_period
            #     )        


        # ------- BEFORE ROW FOLLOW --------
        elif self.state == RobotState.BEFORE_ROW_FOLLOW:

            if self.current_row % 2 == 1: # Odd row (changing at before row change)
                self.line_mode_pub.publish(Int32(data=6))  # NO_LINE_SENSORS
                self.motion_pub.publish(Int32(data=14))     # MOVE RIGHT 
                if (
                        (self.falling_edges[2] == 1 or self.side_wall_detected)
                        and not self.row_change_latched
                    ):  # Assuming L3 is the sensor
                    self.falling_edges = [0,0,0,0] # reset the falling edge
                    self.current_row += 1
                    self.row_change_latched = True
                    self.get_logger().info(f"Reached next row {self.current_row} after row change")
                    self.state = RobotState.ROW_FOLLOW
            
            elif self.current_row % 2 == 0: # Even row (changing at before row change)
                self.line_mode_pub.publish(Int32(data=6))  # NO_LINE_SENSORS
                self.motion_pub.publish(Int32(data=17))     # MOVE RIGHT 
                if (
                        (self.falling_edges[1] == 1 or self.side_wall_detected)
                        and not self.row_change_latched
                    ):  # Assuming L2 is the sensor
                    self.falling_edges = [0,0,0,0] # reset the falling edge
                    self.current_row += 1
                    self.row_change_latched = True
                    self.get_logger().info(f"Reached next row {self.current_row} after row change")
                    self.state = RobotState.ROW_FOLLOW


            #elif not self.before_row_follow_detected:
                # allow future row changes
             #   self.row_change_latched = False



        # -------- FINISHED --------
        elif self.state == RobotState.FINISHED:
            self.get_logger().info("Entered finish state")
            self.motion_pub.publish(Int32(data=12)) # slow backward without line sensors
            self.line_mode_pub.publish(Int32(data=6)) # no line sensors
            if self.end_course:
                self.motion_pub.publish(Int32(data=0))  # STOP
                self.get_logger().info("Navigation complete")
            

    def send_stop(self):
        if self.ser is not None:
            try:
                for _ in range(3):  # send multiple times for reliability
                    self.ser.write(b'<STOP>\n')
                    self.ser.flush()
                    time.sleep(0.1)
                self.get_logger().info("✅ Sent <STOP> to ttyUSB1")
            except Exception as e:
                self.get_logger().error(f"Failed to send STOP: {e}")

    def destroy_node(self):
        # Send STOP before shutting down
        self.send_stop()

        if self.ser is not None:
            try:
                self.ser.close()
                self.get_logger().info("Closed serial port")
            except Exception as e:
                self.get_logger().error(f"Error closing serial: {e}")

        super().destroy_node()


def main():
    rclpy.init()
    node = StateMachineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl+C detected — stopping robot...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()