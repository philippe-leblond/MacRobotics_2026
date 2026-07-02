import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import serial


class Esp32SensorNode(Node):

    def __init__(self):
        super().__init__('esp32_sensor_node')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.latest_line = None
        self.latest_ultra = None

        self.log_timer = self.create_timer(1.0, self.log_sensors)
        
        self.buffer = ""    
        # -------------------------
        # Serial setup
        # -------------------------
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )

            self.get_logger().info(f"Connected to ESP32 on {port}")

        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial connection failed: {e}")
            raise

        # Prevent ESP32 reset
        self.serial.setDTR(False)
        self.serial.setRTS(False)

        # Wait for ESP to stabilize
        import time
        time.sleep(2.0)

        # Clear junk
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        # -------------------------
        # Publishers
        # -------------------------
        self.line_pub = self.create_publisher(
            Int32MultiArray,
            '/line_sensors/raw',
            10
        )

        self.ultrasonic_pub = self.create_publisher(
            Float32MultiArray,
            '/ultrasonic/distances',
            10
        )

        # -------------------------
        # Timer (10 Hz is enough)
        # -------------------------
        self.timer = self.create_timer(0.1, self.read_serial)

    def log_sensors(self):
        if self.latest_line is not None and self.latest_ultra is not None:
            self.get_logger().info(
                f"[1Hz] Line sensors: {self.latest_line} | Ultrasonic: {self.latest_ultra}"
            )
            
    # =========================
    # Serial read + parse
    # =========================
    def read_serial(self):
        if not self.serial.in_waiting:
            return
       
        try:
            # Read ALL available bytes (not line-by-line anymore)
            data = self.serial.read(self.serial.in_waiting or 1).decode('utf-8', errors='ignore')
            self.buffer += data
        except:
            return

        # Process complete messages only
        while '<' in self.buffer and '>' in self.buffer:
            start = self.buffer.find('<')
            end = self.buffer.find('>', start)

            if end == -1:
                break

            line = self.buffer[start:end+1]
            self.buffer = self.buffer[end+1:]

            # ✅ Debug (optional: change to debug later)
            self.get_logger().info(f"Complete line: {line}")

            self.parse_line(line)
    
    def parse_line(self, line):
        try:
            content = line[1:-1]
            parts = content.split('|')

            if len(parts) != 2:
                self.get_logger().warn(f"Unexpected format: {line}")
                return

            line_part = parts[0].strip()
            ultra_part = parts[1].strip()

            # ---- Line sensors ----
            l_values = line_part.replace('L:', '').split(',')
            line_data = [int(v) for v in l_values]

            # ---- Ultrasonic ----
            u_values = ultra_part.replace('U:', '').split(',')
            ultra_data = [float(v) for v in u_values]

            # ✅ Publish
            line_msg = Int32MultiArray()
            line_msg.data = line_data
            self.line_pub.publish(line_msg)

            ultra_msg = Float32MultiArray()
            ultra_msg.data = ultra_data
            self.ultrasonic_pub.publish(ultra_msg)

            # ✅ Save latest
            self.latest_line = line_data
            self.latest_ultra = ultra_data

        except Exception:
            self.get_logger().warn(f"Parse error: {line}")


def main():
    rclpy.init()
    node = Esp32SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()