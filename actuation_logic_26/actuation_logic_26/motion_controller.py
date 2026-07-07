import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
import serial


class MotionControllerNode(Node):

    def __init__(self):
        super().__init__('motion_controller_node')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('port', '/dev/ttyUSB1') #need to change it back to /dev/ttyUSB1 for real robot, for simulation we can use a fake port or just skip the serial connection
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('row_speed', 255)
        self.declare_parameter('slow_row_speed', 180)
        self.declare_parameter('turn_speed', 255)
        self.declare_parameter('slide_speed', 255)
        self.declare_parameter('slow_slide_speed', 180)
    
            # For testing different speeds

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.row_speed = self.get_parameter('row_speed').value
        self.slow_row_speed = self.get_parameter('slow_row_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.slide_speed = self.get_parameter('slide_speed').value
        self.slow_slide_speed = self.get_parameter('slow_slide_speed').value

        # -------------------------
        # Serial connection
        # -------------------------
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to motor ESP32 on {port}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Cannot open serial port: {e}")
            raise

        # -------------------------
        # State
        # -------------------------
        self.motion_mode = 0 # coming from the state machine node, 0 for stop, 1 for line follow forward, 2 for line follow backward, 3 for line follow slide left, 4 for line follow slide right, 5 for move forward, 6 for move backward, 7 for slide left, 8 for slide right, 9 for turn left, 10 for turn right
        self.line_direction = 0 # coming from the line processing node, -1 for left, 0 for straight, +1 for right
        self.side_wall_detected = False # just for debuggin purposes 
        self.plant_pid_output = 0.0 # coming from the ultrasonic processing node, this is the output of the PID controller that tries to keep the robot at a certain distance from the plant
        self.plant_pid_output_camera = 0.0 # coming from the allignment camera node, this is the output of the PID controller that tries to keep the robot centered on the plant

        self.last_command = ""

        # -------------------------
        # Subscriptions
        # -------------------------
        self.create_subscription(
            Int32, '/motion_mode', self.motion_mode_cb, 10)

        self.create_subscription(
            Int32, '/line_direction', self.line_dir_cb, 10)

        #self.create_subscription(Float32, '/plant_pid_output', self.plant_pid_cb, 10)
        self.create_subscription(Float32, '/plant_pid_output_camera', self.plant_pid_camera_cb, 10)
        # -------------------------
        # Control loop
        # -------------------------
        self.timer = self.create_timer(0.1, self.update_motion)

        self.get_logger().info("Motion controller node started")
        self.get_logger().info(
            f"Parameters: row_speed={self.row_speed}, turn_speed={self.turn_speed}, "
            f"slide_speed={self.slide_speed}"
        )

    # =========================
    # Callbacks
    # =========================
    def motion_mode_cb(self, msg):
        old_mode = self.motion_mode
        self.motion_mode = msg.data
        if old_mode != self.motion_mode:
            mode_names = {
                0: "STOP", 1: "LINE_FOLLOW_FORWARD", 2: "LINE_FOLLOW_BACKWARD",
                3: "LINE_FOLLOW_SLIDE_LEFT", 4: "LINE_FOLLOW_SLIDE_RIGHT",
                5: "MOVE_FORWARD", 6: "MOVE_BACKWARD", 7: "SLIDE_LEFT",
                8: "MOVERIGHTODDROW", 9: "TURN_LEFT", 10: "TURN_RIGHT", 
                11: "SLOW_ROW_FORWARD", 12: "SLOW_ROW_BACKWARD", 13: "SLOW_SLIDE_LEFT",
                14: "SLOW_SLIDE_RIGHT", 15: "PID_CONTROL", 16: "MOVERIGHTEVENROW",
                17: "MOVERIGHTEVENROW_SLOW", 18: "LINE_FOLLOW_SLIDE_RIGHT_EVEN_ROW"
            }
            mode_name = mode_names.get(self.motion_mode, f"UNKNOWN({self.motion_mode})")
            self.get_logger().info(f"Motion mode changed: {old_mode} → {self.motion_mode} ({mode_name})")

    def line_dir_cb(self, msg):
        old_direction = self.line_direction
        self.line_direction = msg.data
        if old_direction != self.line_direction:
            dir_names = {-1: "LEFT", 0: "STRAIGHT", 1: "RIGHT"}
            old_name = dir_names.get(old_direction, f"UNKNOWN({old_direction})")
            new_name = dir_names.get(self.line_direction, f"UNKNOWN({self.line_direction})")
            self.get_logger().info(f"Line direction changed: {old_name} → {new_name}")

    def side_wall_cb(self, msg):
        old_detected = self.side_wall_detected
        self.side_wall_detected = msg.data
        if old_detected != self.side_wall_detected:
            self.get_logger().info(f"Side wall detected: {old_detected} → {self.side_wall_detected}")
    
    def plant_pid_cb(self, msg):
        self.plant_pid_output = msg.data
        self.get_logger().info(f"Plant PID output updated: {self.plant_pid_output}")

    def plant_pid_camera_cb(self, msg):
        self.plant_pid_output_camera = msg.data
        self.get_logger().info(f"Plant PID output camera updated: {self.plant_pid_output_camera}")

    # =========================
    # Main control logic
    # =========================
    def update_motion(self):
        cmd = None

        # ---- STOP ----
        if self.motion_mode == 0:
            cmd = "<STOP>"

        # ---- LINE FOLLOW FORWARD ----
        elif self.motion_mode == 1:
            if self.line_direction == 0:
                cmd = f"<FORWARD:{self.row_speed}>"
            elif self.line_direction > 0:
                cmd = f"<CURVECCWFORWARD:{self.turn_speed}>"
            else:
                cmd = f"<CURVECWFORWARD:{self.turn_speed}>"

        # ---- LINE FOLLOW BACKWARD ----
        elif self.motion_mode == 2:
            if self.line_direction == 0:
                cmd = f"<BACKWARD:{self.row_speed}>"
            elif self.line_direction > 0:
                # MIGHT NEED TO  CHANGE TO RIGHT AND  LEFT TO RIGHT
                cmd = f"<CURVECCWBACKWARD:{self.turn_speed}>" #need to test if this should be turn left or right, because the robot goes backward
            else:
                cmd = f"<CURVECWBACKWARD:{self.turn_speed}>" #same as above, need to test if this should be turn left or right
        
        # ---- LINE FOLLOW SLIDE LEFT ----
        elif self.motion_mode == 3:
            if self.line_direction == 0:
                cmd = f"<MOVELEFT:{self.slide_speed}>"
            elif self.line_direction > 0:
                cmd = f"<FORWARD:{self.turn_speed}>" #need to test if this should be turn left or right, because the robot goes backward
            else:
                cmd = f"<BACKWARD:{self.turn_speed}>"
        # ---- LINE FOLLOW SLIDE RIGHT ----
        elif self.motion_mode == 4:
            if self.line_direction == 0:
                cmd = f"<MOVERIGHTODDROW:{self.slide_speed}>"
            elif self.line_direction > 0: #L2 IS 1 AND L3 IS -1
                cmd = f"<CURVECCWMOVERIGHT:{self.turn_speed}>" #L2
            else:
                cmd = f"<CURVECWMOVERIGHT:{self.turn_speed}>" #L3

        # ---- MOVE FORWARD ----
        elif self.motion_mode == 5:
            cmd = f"<FORWARD:{self.row_speed}>"

        # ---- MOVE BACKWARD ----
        elif self.motion_mode == 6:
            cmd = f"<BACKWARD:{self.row_speed}>"

        # ---- SLIDE LEFT ----
        elif self.motion_mode == 7:
            cmd = f"<MOVELEFT:{self.slide_speed}>"


        # ---- SLIDE RIGHT ----
        elif self.motion_mode == 8:
            cmd = f"<MOVERIGHTODDROW:{self.slide_speed}>"

        # ---- TURN LEFT ----
        elif self.motion_mode == 9:
            cmd = f"<TURNCW:{self.turn_speed}>"

        # ---- TURN RIGHT ----
        elif self.motion_mode == 10:
            cmd = f"<TURNCCW:{self.turn_speed}>"
        
        # ---- SLOW ROW FORWARD ----
        elif self.motion_mode == 11:
            cmd = f"<FORWARD:{self.slow_row_speed}>"

        # ---- SLOW ROW BACKWARD ----
        elif self.motion_mode == 12:
            cmd = f"<BACKWARD:{self.slow_row_speed}>"

        # ---- SLOW SLIDE LEFT ----
        elif self.motion_mode == 13:
            cmd = f"<MOVELEFT:{self.slow_slide_speed}>"
        
        # ---- SLOW SLIDE RIGHT ----
        elif self.motion_mode == 14:
            cmd = f"<MOVERIGHTODDROW:{self.slow_slide_speed}>"

        # ---- PID CONTROL ----
        elif self.motion_mode == 15:

            # self.plant_pid_output_camera = 0.0

            speed = 100 + int(abs(self.plant_pid_output_camera) * 255) # Convert to PWM

            speed = int(max(100, min(180, speed))) # the 100 is added because the motors don't move before PWM 100 # Clamp between 100 and 180

            if self.plant_pid_output_camera < 0:
                cmd = f"<FORWARD:{speed}>"
                self.get_logger().info(f"PID Control: Moving Forward with speed {speed} due to plant_pid_output_camera {self.plant_pid_output_camera}")

            elif self.plant_pid_output_camera > 0:
                cmd = f"<BACKWARD:{speed}>"
                self.get_logger().info(f"PID Control: Moving Backward with speed {speed} due to plant_pid_output_camera {self.plant_pid_output_camera}")

            else:
                cmd = "<STOP>"
                self.get_logger().info(f"PID Control: Stopping due to plant_pid_output_camera {self.plant_pid_output_camera}")

        # ---- MOVE RIGHT EVEN ROW ----
        elif self.motion_mode == 16:
            cmd = f"<MOVERIGHTEVENROW:{self.slide_speed}>"
        
        # ---- SLOW MOVE RIGHT EVEN ROW ----
        elif self.motion_mode == 17:
            cmd = f"<MOVERIGHTEVENROW_SLOW:{self.slow_slide_speed}>"

                # ---- LINE FOLLOW SLIDE RIGHT ----
        elif self.motion_mode == 18:
            if self.line_direction == 0:
                cmd = f"<MOVERIGHTEVENROW:{self.slide_speed}>"
            elif self.line_direction > 0: #L2 IS 1 AND L3 IS -1
                cmd = f"<CURVECCWMOVERIGHT:{self.turn_speed}>" #L2
            else:
                cmd = f"<CURVECWMOVERIGHT:{self.turn_speed}>" #L3
            
        # ---- Send only if changed ----
        if cmd and cmd != self.last_command:
            try:
                self.ser.write(cmd.encode('utf-8'))
                self.ser.flush()
                self.last_command = cmd
                self.get_logger().info(f"Sent command: {cmd}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send command '{cmd}': {e}")
        elif cmd is None:
            self.get_logger().warn(f"No command generated for motion_mode {self.motion_mode}")

        # Periodic status logging (every 3 seconds)
        import time
        current_time = time.time()
        if not hasattr(self, 'last_status_log') or current_time - self.last_status_log > 3.0:
            mode_names = {
                0: "STOP", 1: "LINE_FOLLOW_FORWARD", 2: "LINE_FOLLOW_BACKWARD",
                3: "LINE_FOLLOW_SLIDE_LEFT", 4: "LINE_FOLLOW_SLIDE_RIGHT",
                5: "MOVE_FORWARD", 6: "MOVE_BACKWARD", 7: "SLIDE_LEFT",
                8: "SLIDE_RIGHT", 9: "TURN_LEFT", 10: "TURN_RIGHT",
                11: "SLOW_ROW_FORWARD", 12: "SLOW_ROW_BACKWARD", 13: "SLOW_SLIDE_LEFT",
                14: "SLOW_SLIDE_RIGHT", 15: "PID_CONTROL", 16: "MOVERIGHTEVENROW", 17: "MOVERIGHTEVENROW_SLOW",
                18: "LINE_FOLLOW_SLIDE_RIGHT_EVEN_ROW"
            }
            mode_name = mode_names.get(self.motion_mode, f"UNKNOWN({self.motion_mode})")
            dir_names = {-1: "LEFT", 0: "STRAIGHT", 1: "RIGHT"}
            dir_name = dir_names.get(self.line_direction, f"UNKNOWN({self.line_direction})")
            
            self.get_logger().info(
                f"Status: mode={mode_name}, direction={dir_name}, "
                f"side_wall={self.side_wall_detected}, last_cmd='{self.last_command}'"
            )
            self.last_status_log = current_time

def main():
    rclpy.init()
    node = MotionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()