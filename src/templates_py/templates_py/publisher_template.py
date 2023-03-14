#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherTemplate(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.pub = self.create_publisher(
            msg_type=String,
            topic='/example_topic',
            qos_profile=10
        )

        self.timer = self.create_timer(
            timer_period_sec=1.0,
            callback=self.publish
        )

    def publish(self) -> None:
        msg = String()
        msg.data = 'Music'
        
        self.get_logger().info(message='Transmitting: ' + msg.data)

        self.pub.publish(msg=msg)

def main(args=None):
    rclpy.init(args=args)
    publisher_template = PublisherTemplate(node_name='publisher_template')
    rclpy.spin(node=publisher_template)
    rclpy.shutdown()

if __name__ == '__main__':
    main()