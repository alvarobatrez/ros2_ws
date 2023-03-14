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
            callback=self.get_message,
            qos_profile=10
        )

    def get_message(self, msg: String) -> None:
        self.get_logger().info(message='Receiving: ' + msg.data)

def main(args=None):
    rclpy.init(args=args)
    subscriber_template = SubscriberTemplate(node_name='subscriber_template')
    rclpy.spin(node=subscriber_template)
    rclpy.shutdown()

if __name__ == '__main__':
    main()