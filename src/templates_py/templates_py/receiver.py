#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Receiver(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.sub = self.create_subscription(
            msg_type=String,
            topic='/example_topic',
            callback=self.callback,
            qos_profile=10
        )

    def callback(self, msg):
        self.get_logger().info(message=f'Receiving: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    receiver = Receiver(node_name='receiver')
    rclpy.spin(receiver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()