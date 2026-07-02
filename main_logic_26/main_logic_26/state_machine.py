import rclpy
from rclpy.node import Node
from std_msgs import msg
from std_msgs.msg import Int32, Bool, Int32MultiArray
from enum import Enum
import serial
import time


class RobotState(Enum):
    INIT_SLIDE_LEFT = 0 # can add more state for initialization if needed
    INIT_FORWARD_1 = 1
    INIT_CHANGE_ROW = 2
    INIT_BEFORE_ROW_FORWARD = 3
    ROW_FOLLOW = 4
    ROW_SLOW = 5
    PLANT_DETECTED = 6
    PLANT_ACT = 7
    WAIT = 8
    FINISH_ROW = 9
    BEFORE_ROW_CHANGE = 10
    ROW_CHANGE = 11
    BEFORE_ROW_FOLLOW = 12
    FINISH = 13


class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine')

        # ---------- Servo serial ----------
        self.serServo = None
        try:
            self.serServo = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Opened serial port /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        # ---------- Motor serial ----------
        self.serMotor = None
        try:
            self.serMotor = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) # need to change it back on the RPi
            self.get_logger().info("Opened serial port /dev/ttyUSB1")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        # ---------- Publishers ----------
        self.motion_mode_pub = self.create_publisher(Int32, '/motion_mode', 10)
        self.line_mode_pub = self.create_publisher(Int32, '/line_mode', 10)
        self.row_index_pub = self.create_publisher(Int32, '/row_index', 10)
        self.plant_index_pub = self.create_publisher(Int32, '/plant_index', 10)
        self.camera_request_pub = self.create_publisher(Bool, '/camera_request', 10) # NEED TO CREATE IN THE FUTUR CAMERA NODE


        # ---------- Subscriptions ----------
        self.create_subscription(Bool, '/row_end', self.row_end_cb, 10)
        self.create_subscription(Bool, '/forward_pair_black', self.forward_pair_cb, 10)
        self.create_subscription(Bool, '/backward_pair_black', self.backward_pair_cb, 10)
        self.create_subscription(Bool, '/plant_detected', self.plant_cb, 10)
        self.create_subscription(Bool, '/init_slide_wall_1_detected', self.init_slide_1_cb, 10)
        self.create_subscription(Bool, '/init_slide_wall_2_detected', self.init_slide_2_cb, 10)
        # self.create_subscription(Bool, '/init_slide_wall_3_detected', self.init_slide_3_cb, 10)
        self.create_subscription(Bool, '/init_forward_wall_1_detected', self.init_forward_1_cb, 10)
        self.create_subscription(Int32,'/force_state', self.force_state_cb, 10)
        self.create_subscription(Int32, '/camera_choice', self.camera_choice_cb, 10) # NEED TO CREATE IN THE FUTUR CAMERA NODE
        self.create_subscription(Bool, '/before_row_change_detected', self.before_row_change_cb, 10)
        self.create_subscription(Bool, '/before_row_follow_detected', self.before_row_follow_cb, 10)
        # self.create_subscription(Bool, '/row_change_arrival_detected', self.row_change_arrival_cb, 10) 
        self.create_subscription(Int32MultiArray, '/line_falling_edges', self.falling_edges_cb, 10)


        # ---------- Internal state ----------
        self.state = RobotState.INIT_SLIDE_LEFT
        self.row = 1
        self.plant_count = 0
        self._last_log = 0.0 # loggin purpose only
        self._last_plant_log_time = 0.0 # logging purpose only

        self.row_end = False
        self.forward_pair_black = False
        self.backward_pair_black = False
        self.plant_detected = False
        self.init_slide_wall_1_detected = False
        self.init_slide_wall_2_detected = False
        self.init_slide_wall_3_detected = False
        self.init_forward_wall_1_detected = False
        self.on_dashed_line = False
        self.forward_pair_latched = False
        self.backward_pair_latched = False
        self.camera_choice = -1   # -1 = no decision, 0 = LEFT, 1 = RIGHT
        self.camera_request_sent = False
        self.before_row_change_detected = False
        self.before_row_follow_detected = False
        self.row_change_arrival_detected = False
        self.row_change_latched = False
        self.end_course_detected = False
        self.max_rows = 5 # when five it's time to go in the finish and the thresold it >=5
        self.falling_edges = [0,0,0,0]

        
        self.wait_start = None

        self.timer = self.create_timer(0.1, self.step)

        self.get_logger().info("State machine started")

    # ---------- Callbacks ----------
    def row_end_cb(self, msg):
        self.row_end = msg.data

    def forward_pair_cb(self, msg):
        self.forward_pair_black = msg.data

    def backward_pair_cb(self, msg):
        self.backward_pair_black = msg.data
    
    def plant_cb(self, msg):
        self.plant_detected = msg.data

    def init_slide_1_cb(self, msg):
        self.init_slide_wall_1_detected = msg.data

    def init_slide_2_cb(self, msg):
        self.init_slide_wall_2_detected = msg.data

    def init_slide_3_cb(self, msg):
        self.init_slide_wall_3_detected = msg.data

    def init_forward_1_cb(self, msg):
        self.init_forward_wall_1_detected = msg.data
    
    def camera_choice_cb(self, msg):
        self.camera_choice = msg.data

    def before_row_change_cb(self, msg):
        self.before_row_change_detected = msg.data

    def before_row_follow_cb(self, msg):
        self.before_row_follow_detected = msg.data
    
    def row_change_arrival_cb(self, msg):
        self.row_change_arrival_detected = msg.data

    def falling_edges_cb(self, msg):
        self.falling_edges = msg.data

    def force_state_cb(self, msg):
        try:
            forced_state = RobotState(msg.data)
            self.get_logger().warn(
                f"FORCING STATE → {forced_state.name}"
            )
            self.state = forced_state

            # Reset transient state so the newly forced state starts cleanly.
            #self.forward_pair_latched = False
            #self.backward_pair_latched = False
            self.on_dashed_line = False
            self.plant_detected = False
            self.wait_start = None
            self.camera_choice = -1
            self.camera_request_sent = False

        except ValueError:
            self.get_logger().error(
                f"Invalid forced state value: {msg.data}"
            )
    # ---------- Main logic ----------
    def step(self):
        if not self.forward_pair_black and not self.backward_pair_black:
            if self.on_dashed_line:
                self.get_logger().info("Left dashed line, ready for next one")
            self.on_dashed_line = False

        # CAN DO THE LOGIC BELOW IF THE LINE SENSORS ARE FLICKERING TOO MUCH

        #if not self.forward_pair_black and not self.backward_pair_black:
        #    self.no_line_count += 1
        #    if self.no_line_count >= 3:  # 3 cycles × 0.1s = 300 ms
        #        if self.on_dashed_line:
        #           self.get_logger().info("Left dashed line, ready for next one")
        #      self.on_dashed_line = False
        #else:
        #    self.no_line_count = 0

        # DEBUG ROW
        now = time.monotonic()
        if now - self._last_plant_log_time >= 1.0:
            self.get_logger().info(
                f"[STATE_MACHINE] row={self.row} "
                f"plant_count={self.plant_count} "
                f"state={self.state.name}"
            )
            self._last_plant_log_time = now


        # ===== INIT: SLIDE LEFT =====
        if self.state == RobotState.INIT_SLIDE_LEFT:
            self.line_mode_pub.publish(Int32(data=6))   # NO LINE DETECTION
            self.motion_mode_pub.publish(Int32(data=14))      # SLOW SLIDE RIGHT

            if self.init_slide_wall_1_detected:
                self.get_logger().info("Init slide 1 complete")
                self.state = RobotState.INIT_FORWARD_1

        # ===== INIT: MOVE FORWARD =====
        elif self.state == RobotState.INIT_FORWARD_1:
            self.motion_mode_pub.publish(Int32(data=5))  # MOVE FORWARD
            self.line_mode_pub.publish(Int32(data=6))  # NO LINE DETECTION
        
            if self.falling_edges[1] == 1:  # Assuming L2 is the sensor
                self.falling_edges = [0,0,0,0] # reset the falling edge
                self.get_logger().info("Init move forward complete")
                self.state = RobotState.INIT_CHANGE_ROW
        
        # ===== INIT: ROW CHANGE =====
        elif self.state == RobotState.INIT_CHANGE_ROW:
            self.motion_mode_pub.publish(Int32(data=4))  # SLIDE RIGHT
            self.line_mode_pub.publish(Int32(data=5))    # L1/L4

            if self.init_slide_wall_2_detected: #NEED TO ADD BEFORE ROW FOLLOW DETECTION
                self.get_logger().info("Init slide 2 complete")
                self.state = RobotState.INIT_BEFORE_ROW_FORWARD
        
        #==== INIT: BEFORE ROW FORWARD =====
        elif self.state == RobotState.INIT_BEFORE_ROW_FORWARD:
            self.motion_mode_pub.publish(Int32(data=14))  # SLIDE RIGHT SLOW
            self.line_mode_pub.publish(Int32(data=6))  # NO LINE DETECTION

            if self.falling_edges[1] == 1:  # Assuming L2 is the sensor
                self.falling_edges = [0,0,0,0] # reset the falling edge
                self.get_logger().info("Initialization complete")
                self.state = RobotState.ROW_FOLLOW
        
       

        # ==================================================
        # ROW FOLLOW
        # ==================================================
        elif self.state == RobotState.ROW_FOLLOW:
            self.row_index_pub.publish(Int32(data=self.row))
            self.row_change_latched = False #reset row change latch at the beginning of each row follow to avoid issues with the row change detection in the ultrasonic node and the row change process in general
            if self.row % 2 == 1:
                self.line_mode_pub.publish(Int32(data=0))  # ROW_FOLLOW_ODD
                self.motion_mode_pub.publish(Int32(data=1))     # LINE FOLLOW FORWARD
            else:
                self.line_mode_pub.publish(Int32(data=2))  # ROW_FOLLOW_EVEN
                self.motion_mode_pub.publish(Int32(data=2))     # LINE FOLLOW BACKWARD

            if self.row % 2 == 1:
                if self.forward_pair_black and not self.on_dashed_line:
                    self.on_dashed_line = True
                    self.get_logger().info("Forward dashed line detected")
                    self.state = RobotState.ROW_SLOW
            else:
                if self.backward_pair_black and not self.on_dashed_line:
                    self.on_dashed_line = True
                    self.get_logger().info("Backward dashed line detected")
                    self.state = RobotState.ROW_SLOW


        # ==================================================
        # ROW SLOW
        # ================================================== 
        # need to test the threshold between the moment where the robot waits 1 second for a plant goes back to the row_odd_run because now its going directly to thee row odd slow since it's directly [1,1,0,0]
        elif self.state == RobotState.ROW_SLOW:


            if self.row % 2 == 1:
                self.motion_mode_pub.publish(Int32(data=11))  # slow forward
                self.line_mode_pub.publish(Int32(data=6))     # NO LINE SENSORS
            else:
                self.motion_mode_pub.publish(Int32(data=12))  # slow backward   
                self.line_mode_pub.publish(Int32(data=6))     # NO LINE SENSORS
            
            # ultrasonic node stops us using plant thresholds
            if self.plant_detected:
                self.motion_mode_pub.publish(Int32(data=0))  # STOP robot

                # Reset state for new decision
                self.camera_choice = -1
                self.camera_request_sent = False

                self.wait_start = time.time()
                self.state = RobotState.PLANT_DETECTED
        

        # ==================================================
        # PLANT DETECTED
        # ================================================== 
        elif self.state == RobotState.PLANT_DETECTED:

            # Send request ONLY ONCE
            if not self.camera_request_sent:
                self.camera_request_pub.publish(Bool(data=True))
                self.camera_request_sent = True # look in the camera node if the camera_request vaariable will change to false after sending
                self.get_logger().info("Camera request sent")

            # Wait for camera decision
            if self.camera_choice != -1:

                if self.camera_choice == 0:
                    self.get_logger().info("Camera says LEFT")
                    self.state = RobotState.PLANT_ACT
                elif self.camera_choice == 1:
                    self.get_logger().info("Camera says RIGHT")
                    self.state = RobotState.PLANT_ACT
                else:
                    self.get_logger().info("No plants to remove → Don't activate servo")
                    self.camera_choice = -1  # reset to avoid issues
                    self.camera_request_sent = False
                    self.state = RobotState.WAIT # just wait 1 second and continue the planting process and skip the plant actuation since there are no plants
                

            # Timeout fallback (VERY important)
            #elif time.time() - self.wait_start > 0.5:   
            #self.get_logger().warn("Camera timeout → ski
            #self.camera_choice = -1
            #self.camera_request_sent = False

            #    self.state = RobotState.WAIT

        # ==================================================
        # PLANT ACT
        # ==================================================
        elif self.state == RobotState.PLANT_ACT:

            if self.camera_choice == 0:
                # LEFT
                if self.serServo:
                    self.serServo.write(b"servo1\n")
                self.get_logger().info("Executing LEFT plant (servo1)")

            elif self.camera_choice == 1:
                # RIGHT
                if self.serServo:
                    self.serServo.write(b"servo2\n")
                self.get_logger().info("Executing RIGHT plant (servo2)")

            else:
                self.get_logger().warn("No valid camera choice → no action")

            # RESET EVERYTHING (critical)
            self.camera_choice = -1
            self.camera_request_sent = False

            self.wait_start = time.time()
            self.state = RobotState.WAIT

  
        # ==================================================
        # WAIT AFTER PLANT
        # ==================================================
        elif self.state == RobotState.WAIT:
            if time.time() - self.wait_start >= 1.0:
                self.plant_count += 1
                self.plant_index_pub.publish(Int32(data=self.plant_count))

                if self.plant_count < 6:
                    self.state = RobotState.ROW_FOLLOW
                else:
                    self.state = RobotState.FINISH_ROW


        # ==================================================
        # FINISH ROW
        # ==================================================
        elif self.state == RobotState.FINISH_ROW:

            if self.row % 2 == 1:
                # Forward row
                self.motion_mode_pub.publish(Int32(data=1))
                self.line_mode_pub.publish(Int32(data=0))
            else:
                # Backward row
                self.motion_mode_pub.publish(Int32(data=2))
                self.line_mode_pub.publish(Int32(data=2))

            if self.before_row_change_detected:
                self.get_logger().info(f"Row {self.row} finished")

                self.plant_count = 0
                self.plant_index_pub.publish(Int32(data=self.plant_count))
                
                # Decide next state
                if self.row >= self.max_rows:
                    self.state = RobotState.FINISH # COULD CHANGE THE CONDITION FOR FINISH IN THE BEFORE ROW CHANGE STATE
                else:
                    #self.state = RobotState.ROW_FOLLOW
                    self.get_logger().info("Continue to next row, preparing for before row change")
                    self.state = RobotState.BEFORE_ROW_CHANGE

        # ==================================================
        # BEFORE ROW CHANGE 
        # ==================================================
        elif self.state == RobotState.BEFORE_ROW_CHANGE:

            if self.row % 2 == 1: # ODD
                self.line_mode_pub.publish(Int32(data=6))  # NO_LINE_SENSORS
                self.motion_mode_pub.publish(Int32(data=11))     # SLOW FORWARD
                if self.falling_edges[0] == 1:  # Assuming L1 is the sensor
                    self.falling_edges = [0,0,0,0] # reset the falling edge            
                    self.get_logger().info("Going to change row from slow forward")
                    self.state = RobotState.ROW_CHANGE
            else: # EVEN
                self.line_mode_pub.publish(Int32(data=6))  # NO_LINE_SENSORS
                self.motion_mode_pub.publish(Int32(data=12))     # SLOW BACKWARD
                if self.falling_edges[3] == 1:  # Assuming L4 is the sensor
                    self.falling_edges = [0,0,0,0] # reset the falling edge            
                    self.get_logger().info("Going to change row from slow backward")
                    self.state = RobotState.ROW_CHANGE
            # if self.row_end:
            #     self.get_logger().info("Switching to ROW_CHANGE")
            #     self.state = RobotState.ROW_CHANGE

        # ==================================================
        # ROW CHANGE 
        # ==================================================
        elif self.state == RobotState.ROW_CHANGE:

            # From forward row → go RIGHT
            self.motion_mode_pub.publish(Int32(data=4)) # RIGHT ROW CHANGE
            self.line_mode_pub.publish(Int32(data=4)) # SLIDE RIGHT

            if self.before_row_follow_detected:  # reuse a valid sensor
                self.get_logger().info("Row change complete")
                self.state = RobotState.BEFORE_ROW_FOLLOW
        
        # ==================================================
        # BEFORE ROW CHANGE 
        # ==================================================
        elif self.state == RobotState.BEFORE_ROW_FOLLOW:
            self.line_mode_pub.publish(Int32(data=6))  # NO_LINE_SENSORS
            self.motion_mode_pub.publish(Int32(data=14))     # MOVE RIGHT 

            if self.row % 2 == 1: # Odd row (changing at before row change)
                if self.falling_edges[2] == 1 and not self.row_change_latched:  # Assuming L3 is the sensor
                    self.falling_edges = [0,0,0,0] # reset the falling edge
                    self.row += 1
                    self.row_change_latched = True
                    self.get_logger().info(f"Reached next row {self.row} after row change")
                    self.state = RobotState.ROW_FOLLOW
            
            elif self.row % 2 == 0: # Even row (changing at before row change)
                if self.falling_edges[1] == 1 and not self.row_change_latched:  # Assuming L2 is the sensor
                    self.falling_edges = [0,0,0,0] # reset the falling edge
                    self.row += 1
                    self.row_change_latched = True
                    self.get_logger().info(f"Reached next row {self.row} after row change")
                    self.state = RobotState.ROW_FOLLOW


        # ==================================================
        # FINISH
        # ==================================================
        elif self.state == RobotState.FINISH:
            self.motion_mode_pub.publish(Int32(data=5))  # STOP
            self.line_mode_pub.publish(Int32(data=6))    # NO LINE DETECTION
            if self.end_course_detected: #define this variable in the ultrasonic node with the specific threshold for the end of the course
                self.motion_mode_pub.publish(Int32(data=0))  # STOP
                self.line_mode_pub.publish(Int32(data=6))    # NO LINE DETECTION
                self.get_logger().info("All rows completed! Robot stopped.")
       

    def send_stop(self):
        if self.serMotor is not None:
            try:
                for _ in range(3):  # send multiple times for reliability
                    self.serMotor.write(b'<STOP>\n')
                    self.serMotor.flush()
                    time.sleep(0.1)
                self.get_logger().info("✅ Sent <STOP> to ttyUSB1")
            except Exception as e:
                self.get_logger().error(f"Failed to send STOP: {e}")

    def destroy_node(self):
        # Send STOP before shutting down
        self.send_stop()

        if self.serMotor or self.serServo is not None:
            try:
                self.serMotor.close()
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