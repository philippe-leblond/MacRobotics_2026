#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from gpiozero import LED

LED_ON_TIME = 5.0   # seconds

class LedProcessingNode(Node):

    def __init__(self):
        super().__init__('led_subscriber')

        self.leds = {
            0: LED(21),  # pin 40
            1: LED(20),  # pin 38
            2: LED(16)   # pin 36
        }

        self.off_timer = None

        self.subscription = self.create_subscription(
            Int32,
            'led_control',
            self.listener_callback,
            10
        )

        self.get_logger().info("LED Subscriber Started")

    def turn_off_all_leds(self):
        for led in self.leds.values():
            led.off()

    def timer_callback(self):
        self.turn_off_all_leds()

        if self.off_timer is not None:
            self.off_timer.cancel()
            self.off_timer = None

        self.get_logger().info("LED automatically turned off")

    def listener_callback(self, msg):
        led_id = msg.data
        self.get_logger().info(f"Received: {led_id}")

        # Turn off any current LEDs
        self.turn_off_all_leds()

        # Cancel previous timer if one exists
        if self.off_timer is not None:
            self.off_timer.cancel()
            self.off_timer = None

        if led_id == -1:
            self.get_logger().info("All LEDs OFF")
            return

        if led_id in self.leds:
            self.leds[led_id].on()

            # Schedule automatic shutoff
            self.off_timer = self.create_timer(
                LED_ON_TIME,
                self.timer_callback
            )


def main(args=None):
    rclpy.init(args=args)
    node = LedProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    for led in node.leds.values():
        led.off()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()