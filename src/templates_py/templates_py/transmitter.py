#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Transmitter(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.declare_parameter(name="period", value=1.0)

        period = self.get_parameter(name="period").value

        self.pub = self.create_publisher(msg_type=String, topic='/example_topic', qos_profile=10)

        self.timer = self.create_timer(timer_period_sec=period, callback=self.publish_callback)

    def publish_callback(self):
        msg = String()
        msg.data = "music"
        self.pub.publish(msg=msg)

        self.get_logger().info(message=f'Transmitting: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    transmitter = Transmitter(node_name='transmitter')
    rclpy.spin(node=transmitter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()