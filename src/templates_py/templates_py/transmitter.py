#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Transmitter(Node):

    def __init__(self, node_name) -> None:
        super().__init__(node_name)

        self.declare_parameter("period", 1.0)

        period = self.get_parameter("period").value

        self.pub = self.create_publisher(msg_type=String, topic='/example_topic', qos_profile=10)

        self.timer = self.create_timer(timer_period_sec=period, callback=self.publish_callback)

    def publish_callback(self):
        msg = String()
        msg.data = "music"
        self.pub.publish(msg)

        self.get_logger().info(f'Transmitting: {msg.data}')

def main(args=None):
    rclpy.init(args=None)
    transmitter = Transmitter('transmitter')
    rclpy.spin(transmitter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()