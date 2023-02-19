#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberTemplate(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.sub = self.create_subscription(
            msg_type=String,
            topic='/example_topic',
            callback=self.callback,
            qos_profile=10
        )

    def callback(self, msg):
        self.get_logger().info(message='Receiving: ' + msg.data)

def main(args=None):
    rclpy.init(args=args)
    subscriber_template = SubscriberTemplate('subscriber_template')
    rclpy.spin(node=subscriber_template)
    rclpy.shutdown()

if __name__ == '__main__':
    main()