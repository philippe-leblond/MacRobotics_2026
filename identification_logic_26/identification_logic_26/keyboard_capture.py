#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class KeyboardCaptureNode(Node):

    def __init__(self):
        super().__init__('keyboard_capture')

        self.pub = self.create_publisher(
            Bool,
            '/capture_trigger',
            10
        )

        self.get_logger().info(
            'Press ENTER whenever a plant is in front of the camera.'
        )

    def run(self):

        while rclpy.ok():

            input()

            msg = Bool()
            msg.data = True

            self.pub.publish(msg)

            self.get_logger().info('Picture requested')


def main(args=None):

    rclpy.init(args=args)

    node = KeyboardCaptureNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()